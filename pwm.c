
#include <linux/module.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/gpio.h>

#include "motor.h"

//  PWM Control regs.

static const PWM pwm[2] = 
{
    {   PWMREG(0x40b00000), PWMREG(0x40b00004), PWMREG(0x40b00008), 16, CKEN0_PWM0 }, // PWM0
    {   PWMREG(0x40b00010), PWMREG(0x40b00014), PWMREG(0x40b00018), 17, CKEN1_PWM1 }, // PWM1
};

//  PwmMotor control

static inline int pwm_get_counter(Motor* m)
{
    return m->m.pwm.position;
}

static inline void pwm_motor_mark(PwmMotor* motor, int value)
{
    if (value < 0)
    {
        gpio_set_value(motor->md.gpio_dir, 1);
        pwm_dcr(motor->pwm, motor->mark = motor->period + value);
    }
    else
    {
        gpio_set_value(motor->md.gpio_dir, 0);
        pwm_dcr(motor->pwm, motor->mark = value);
    }
}

static inline void pwm_motor_off(PwmMotor* motor)
{
    // clear the outputs
    gpio_set_value(motor->md.gpio_en, 0);
    pwm_motor_mark(motor, 0);
    gpio_set_value(motor->md.gpio_dir, 0);
}

#define SENSE_IRQ 0
#define CLOCK_IRQ 1

static int pwm_motor_open(Motor* m)
{
    PwmMotor* motor = & m->m.pwm;
    pwm_motor_off(motor);

    //  Interrupt on the input pin
    m->irqs[SENSE_IRQ] = IRQ_GPIO(motor->md.gpio_clock);
    printk("pwm sense irq = %d\n", m->irqs[SENSE_IRQ]);

    if (request_irq(m->irqs[SENSE_IRQ], on_irq, IRQF_DISABLED, "motor", THIS_MODULE))
    {
        printk("unable to allocate motor sense irq\n");
        return false;
    }

    //  Interrupt on timer
    m->irqs[CLOCK_IRQ] = TIMER_IRQ;
    printk("pwm timer irq = %d\n", m->irqs[SENSE_IRQ]);

    if (request_irq(m->irqs[CLOCK_IRQ], on_irq, IRQF_DISABLED, "motor", THIS_MODULE))
    {
        printk("unable to allocate motor timer irq\n");
        return false;
    }

    motor->timer = timer_open(0xCA); // 1ms periodic
    timer_period(motor->timer, 100);
    timer_enable(motor->timer);

    GEDR(motor->md.gpio_clock) &= ~GPIO_bit(motor->md.gpio_clock);  //  clear any pending edge detects
    GRER(motor->md.gpio_clock) |= GPIO_bit(motor->md.gpio_clock);   // enable rising edge detect

    m->is_open = true;
    gpio_set_value(motor->md.gpio_en, 1);

    return 0;
}

static void pwm_motor_close(Motor* m)
{
    PwmMotor* motor = & m->m.pwm;

    pwm_motor_off(motor);
    m->is_open = false;

    timer_close(motor->timer);

    GRER(motor->md.gpio_clock) &= ~GPIO_bit(motor->md.gpio_clock); // disable rising edge detect

    free_irq(m->irqs[SENSE_IRQ], THIS_MODULE);
    m->irqs[SENSE_IRQ] = 0;
    free_irq(m->irqs[CLOCK_IRQ], THIS_MODULE);
    m->irqs[CLOCK_IRQ] = 0;
}

static bool pwm_on_irq(Motor* m, int index)
{
    PwmMotor* motor = & m->m.pwm;

    if (index == SENSE_IRQ)
    {
        //  Update position depending on direction of travel
        if (gpio_get_value(motor->md.gpio_sense))
            motor->position += 1;
        else
            motor->position -= 1;
        return true;
    }

    if (index == CLOCK_IRQ)
    {
        if (timer_on_irq(motor->timer))
        {
            //  TODO
            return true;
        }
    }

    return false;
}

static void pwm_set_counter(Motor* m, int count)
{
    m->m.pwm.position = count;
}

static void pwm_set_speed(Motor* m, int v)
{
    printk("pwm_set_speed(%d)\n", v);
    m->speed = v;
    pwm_motor_mark(& m->m.pwm, v);
}

static int pwm_get_speed(Motor* m)
{
    return m->speed;
}

static void pwm_move_to(Motor* m, int v)
{
    printk("pwm_move_to() not implemented\n");
}

static bool pwm_config(Motor* m, const char* config)
{
    printk("pwm_config() not implemented\n");
    return false;
}

int pwm_motor_init(Motor* m, const char* name, const void* init)
{
    const PwmDesc* md = (const PwmDesc*) init;
    PwmMotor* motor = & m->m.pwm;

    memset(motor, sizeof(motor), 0);
    memcpy(& motor->md, md, sizeof(PwmDesc));
    m->name = name;

    //  Configure output pins
    gpio_set_value(md->gpio_dir, 0);
    GPIO_mode(md->gpio_dir, GPIO_FN, true);
    gpio_set_value(md->gpio_en, 0);
    GPIO_mode(md->gpio_en, GPIO_FN, true);
    //  Configure input pins
    GPIO_mode(md->gpio_clock, GPIO_FN, false);
    GPIO_mode(md->gpio_sense, GPIO_FN, false);

    //  Configure PWM output
    if (md->pwm_chan != -1)
    {
        motor->pwm = & pwm[md->pwm_chan];
        GPIO_mode(motor->pwm->gpio, ALT_FN_2, true);
        gpio_set_value(motor->pwm->gpio, 1);

        CKEN |= motor->pwm->ck_mask; // enable the PWM clock source
        printk("motor:init: cken = 0x%x\n", CKEN);

        // Set the PWM outputs
        printk("setup PWM chan %d\n", md->pwm_chan);
        motor->pwm = & pwm[md->pwm_chan];
        pwm_pcr(motor->pwm, motor->period = 1023); // 256);
        pwm_cr(motor->pwm, 16);      //  prescaler

        pwm_motor_mark(motor, 0);
    }

    m->open = pwm_motor_open;
    m->close = pwm_motor_close;
    m->get_counter = pwm_get_counter;
    m->set_counter = pwm_set_counter;
    m->on_irq = pwm_on_irq;
    m->set_speed = pwm_set_speed;
    m->get_speed = pwm_get_speed;
    m->move_to = pwm_move_to;
    m->config = pwm_config;

    return 0;
}

// FIN
