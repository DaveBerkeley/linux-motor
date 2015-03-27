
#include <linux/module.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/gpio.h>

#include "motor.h"

    /*  
     *  Stepper Motor
     */

static int stepper_open(struct Motor* m)
{
    printk("stepper_open\n");

    m->irqs[0] = TIMER_IRQ;
    printk("stepper irq=%d\n", m->irqs[0]);

    if (request_irq(m->irqs[0], on_irq, IRQF_DISABLED, "motor", THIS_MODULE))
    {
        printk("unable to allocate motor irq\n");
        return false;
    }

    m->m.stepper.timer = timer_open(0xC9); // 1/32786s periodic

    gpio_set_value(m->m.stepper.desc.gpio_en, 1); // enable the output
    m->m.stepper.state = STOP;

    return 0;
}

static void stepper_close(struct Motor* m)
{
    const int timer = m->m.stepper.timer;
    printk("stepper_close\n");

    gpio_set_value(m->m.stepper.desc.gpio_en, 0); // disable the output
    timer_close(timer);

    free_irq(m->irqs[0], THIS_MODULE);
    m->irqs[0] = 0;
}

static void stepper_set_counter(struct Motor* motor, int count)
{
    printk("stepper_set_counter\n");
    //  TODO
    motor->m.stepper.position = count;
}

static int stepper_get_counter(struct Motor* motor)
{
    return motor->m.stepper.position;
}

static inline void stepper_set_v(StepperMotor* motor, int v)
{
    timer_period(motor->timer, motor->v = v);
}

static void step_state(StepperMotor* motor)
{
    switch(motor->state)
    {
        case ACCEL  :
                {
                    int v = motor->v - motor->accel;
                    if (v < motor->vmax)
                        v = motor->vmax;
                    stepper_set_v(motor, v);
                    if (motor->v == motor->vmax)
                    {
                        motor->state = MOVE;
                        printk("state = MOVE\n");
                        printk("decl_point=%d\n", motor->decel_point);
                    }
                    else if (motor->position == motor->mid_point)
                    {
                        motor->state = DECEL;
                        printk("state = DECEL\n");
                    }
                    break;
                }
        case MOVE   :
                {
                    if (motor->position == motor->decel_point)
                    {
                        motor->state = DECEL;
                        printk("state = DECEL\n");
                    }
                    break;
                }
        case DECEL  :
                {
                    int v = motor->v + motor->accel;
                    if (v > motor->vmin)
                        v = motor->vmin;
                    stepper_set_v(motor, v);
                    if (motor->position == motor->target)
                    {
                        motor->state = STOP;
                        OIER &= ~(1 << motor->timer); // disable irq
                        printk("state = STOP. DONE!\n");
                    }
                    break;
                }
    }
}

static bool stepper_on_irq(struct Motor* m, int index)
{
    StepperMotor* motor = & m->m.stepper;
    const bool me = timer_on_irq(motor->timer);

    if (me && (motor->state != STOP))
    {
        const int was = motor->position;
        int posn = was;

        if (motor->position < motor->target)
            posn += 1;
        else if (motor->position > motor->target)
            posn -= 1;

        if (motor->req_v)
        {
            stepper_set_v(motor, motor->req_v);
            motor->req_v = 0;
        }

        if (motor->position != motor->target)
        {
            switch(posn & 0x03)
            {
                //  TODO : simplify this : only need to change one line each irq
                case 0 : 
                    gpio_set_value(motor->desc.gpio_a, 1);
                    gpio_set_value(motor->desc.gpio_nega, 0);
                    gpio_set_value(motor->desc.gpio_b, 0);
                    gpio_set_value(motor->desc.gpio_negb, 1);
                    break;
                case 1 : 
                    gpio_set_value(motor->desc.gpio_a, 1);
                    gpio_set_value(motor->desc.gpio_nega, 0);
                    gpio_set_value(motor->desc.gpio_b, 1);
                    gpio_set_value(motor->desc.gpio_negb, 0);
                    break;
                case 2 : 
                    gpio_set_value(motor->desc.gpio_a, 0);
                    gpio_set_value(motor->desc.gpio_nega, 1);
                    gpio_set_value(motor->desc.gpio_b, 1);
                    gpio_set_value(motor->desc.gpio_negb, 0);
                    break;
                case 3 : 
                    gpio_set_value(motor->desc.gpio_a, 0);
                    gpio_set_value(motor->desc.gpio_nega, 1);
                    gpio_set_value(motor->desc.gpio_b, 0);
                    gpio_set_value(motor->desc.gpio_negb, 1);
                    break;
            }
            motor->position = posn;
        }

        step_state(motor);
    }

    return me;
}

static void stepper_set_speed(Motor* motor, int v)
{
    stepper_set_v(& motor->m.stepper, v);
}

static int stepper_get_speed(Motor* m)
{
    return m->speed;
}

static void stepper_move_to(Motor* m, int p)
{
    StepperMotor* motor = & m->m.stepper;

    if (!motor->vmin)
    {
        printk("move_to ERROR: no vmin set\n");
        return;
    }

    printk("move to %d, state=ACCEL, v=%d\n", p, motor->vmin);
    motor->target = p;

    // Start the move
    motor->state = ACCEL;
    motor->start = motor->position;
    motor->mid_point = (motor->position - motor->target) / 2;
    if (motor->target > motor->position)
        motor->decel_point = motor->target - ((motor->vmin - motor->vmax) / motor->accel);
    else
        motor->decel_point = motor->target + ((motor->vmin - motor->vmax) / motor->accel);
    stepper_set_v(motor, motor->vmin);

    timer_enable(motor->timer);
}

static bool stepper_config(Motor* motor, const char* config)
{
    int accel, vmin, vmax;

    if (sscanf(config, "%d,%d,%d", & accel, & vmin, & vmax) != 3)
        return false;

    motor->m.stepper.accel = accel;
    motor->m.stepper.vmin = vmin;
    motor->m.stepper.vmax = vmax;
    printk("stepper_config: accel=%d,vmin=%d,vmax=%d\n", accel, vmin, vmax);
    return true;
}

int stepper_init(struct Motor* m, const char* name, const void* init)
{
    StepperMotor* motor = & m->m.stepper;
    const StepperDesc* desc = (const StepperDesc*) init;

    printk("stepper_init\n");
    memset(motor, 0, sizeof(StepperMotor));
    memcpy(& motor->desc, desc, sizeof(StepperDesc));
    m->name = name;

    //  Set the output pins
    GPIO_mode(motor->desc.gpio_a, GPIO_FN, 1);
    GPIO_mode(motor->desc.gpio_nega, GPIO_FN, 1);
    GPIO_mode(motor->desc.gpio_b, GPIO_FN, 1);
    GPIO_mode(motor->desc.gpio_negb, GPIO_FN, 1);
    GPIO_mode(motor->desc.gpio_en, GPIO_FN, 1);

    m->open = stepper_open;
    m->close = stepper_close;
    m->set_counter = stepper_set_counter;
    m->get_counter = stepper_get_counter;
    m->on_irq = stepper_on_irq;
    m->set_speed = stepper_set_speed;
    m->get_speed = stepper_get_speed;
    m->move_to = stepper_move_to;
    m->config = stepper_config;

    return 0;
}

// FIN
