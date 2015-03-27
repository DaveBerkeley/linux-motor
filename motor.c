/*
 *  Motor driver. (C) Dave Berkeley 2007
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <asm/hardware.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <asm/arch/pxa-regs.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>

#include "motor.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Dave Berkeley");
MODULE_DESCRIPTION("Motor control module");

#define DEBUG   1

// Interrupt

static const int GPIO_IRQ_MASK = 1 << 10; // GPIOs 2..x

    //  GPIO Mode control

void GPIO_mode(int gpio, int mode, bool out)
{
    const int shift = (gpio & 0x0F) << 1;
    const int mask = 0x03 << shift;

    if ((mode < 0) || (mode > 3))
    {
        printk("GPIO_mode: Bad mode 0x%x\n", mode);
        return;
    }

    //  Set alt function reg.
    GAFR(gpio) |= (GAFR(gpio) & ~mask) + (mode << shift);

    //  Set direction reg.
    if (out)
        GPDR(gpio) |= GPIO_bit(gpio);
    else
        GPDR(gpio) &= ~GPIO_bit(gpio);
}

    /*
     *  Static Data
     */

static Motor motors[4];
static const int num_motors = 4;

    /*
     *  Irqs
     */

irqreturn_t on_irq(int irq, void* dev_id)
{
    bool handled = false;
    int i;
    int j;
    Motor* motor = motors;

    for (i = 0; i < num_motors; ++i, ++motor)
        for (j = 0; j < MAX_IRQS; j++)
            if (motor->irqs[j] == irq)
                if (motor->on_irq)
                    if (motor->on_irq(motor, j))
                        handled = true;

    return handled ? IRQ_HANDLED : IRQ_NONE;
}

    /*
     *  Timer utils
     */

int timer_open(int mode)
{
    int timer;

    // search thru possible timers for an available one.
    for (timer = 4; timer < 12; ++timer)
    {
        if (!OMCR(timer))
        {
            printk("Allocating timer %d:0x%x\n", timer, mode);
            OMCR(timer) = mode;
            return timer;
        }
    }

    printk("motors: Unable to allocate timer\n");
    return 0;
}

void timer_close(int timer)
{
    printk("close timer %d\n", timer);
    OMCR(timer) = 0; // off
    OIER &= ~(1 << timer); // disable irq
}

void timer_enable(int timer)
{
    OSCR(timer) = 0; // start it
    OIER |= 1 << timer; // enable irq
}

void timer_period(int timer, int period)
{
    OSMR(timer) = period;
}

bool timer_on_irq(int timer)
{
    const int mask = 1 << timer;
    const bool me = OSSR & mask; // was it for me?

    OSSR = mask; // clear the irq
    return me;
}

    /*
     *  CLI Commands
     */

static bool cmd_i(Motor* motor, void(*fn)(Motor*,int), const char* args)
{
    int arg;

    if (!args)
        return false;

    if (sscanf(args, "%d", & arg) != 1)
        return false;

    fn(motor, arg);
    return true;
}

static bool velocity(Motor* motor, char* args)
{
    return cmd_i(motor, motor->set_speed, args);
}

static bool move_to(Motor* motor, char* args)
{
    return cmd_i(motor, motor->move_to, args);
}

static bool config(Motor* motor, char* args)
{
    return motor->config(motor, args);
}

static bool posn(Motor* motor, char* args)
{
    return cmd_i(motor, motor->set_counter, args);
}

    /*
     *  Command intepreter 
     */

extern bool help(Motor* motor, char* args);

typedef bool (*CmdHandler)(Motor* motor, char* args);

typedef struct 
{
    const char* cmd;
    CmdHandler  handler;
    const char* help;
}   CmdTable;

static CmdTable cmd_table[] = 
{
    {   "V",    velocity,   "[V]elocity v" },
    {   "M",    move_to,    "[M]ove_to p" },
    {   "C",    config,     "[C]config (device dependant args...)" },
    {   "P",    posn,       "[P]osition p - force position" },
    {   "?",    help,       "Help" },
    {   0,  },
};

bool help(Motor* motor, char* args)
{
    CmdTable*   cmds;

    printk("Help for commands:\n");

    for (cmds = cmd_table; cmds->cmd; ++cmds)
        printk("%s: %s\n", cmds->cmd, cmds->help);

    return true;
}

static bool exec_command(Motor* motor, char* cmd)
{
    //  passed a \n terminated command in the form:
    //  "COMMAND arg[,arg]...\n" (ie. space between cmd and args)

    CmdTable*   cmds;
    char* space = strchr(cmd, ' ');

    if (!space)
        // terminate the zero-args command
        cmd[strlen(cmd)-1] = '\0';
    else
        // split the cmd and args
        *space++ = '\0'; 

    if (!strlen(cmd)) // null command
        return true;

    for (cmds = cmd_table; cmds->cmd; cmds++)
        if (!strcmp(cmds->cmd, cmd))
            return (cmds->handler)(motor, space);

    printk("Unknown command '%s'\n", cmd);
    return false;
}

static void do_command(Motor* motor, const char* buff, int bytes)
{
    static char line_buff[128];
    int index = strlen(line_buff);
    char* end = & line_buff[index];

    //  Split into \n separated lines, and execute.
    while (bytes--)
    {
        const char c = *end++ = *buff++;

        if (c == '\n')
        {
            *end = '\0';
            if (exec_command(motor, line_buff))
                printk("OK\n");
            else
                printk("error\n");
            index = 0;
            end = line_buff;
            *end = '\0';
        }
        else
        {
            if (++index >= sizeof(line_buff))
            {
                printk("command line overflow!\n");
                break;
            }
        }
    }
}

    /*
     *  File Interface. Common to Motor types.
     */

static int file_open(struct inode* node, struct file* f)
{
    int err;
    //  look up inode.minor to see which motor is being opened.
    const int index = MINOR(node->i_rdev);
    Motor* motor = & motors[index];

    printk(KERN_INFO "motor_open %s\n", motor->name);

    f->private_data = motor;

    err = motor->open(motor);
    if (err)
    {
        printk("Error opening motor: %d\n", err);
        return err;
    }

    return 0;
}

static int file_release(struct inode* x, struct file* f)
{
    Motor* motor = (Motor*) f->private_data;

    printk(KERN_INFO "motor_release\n");

    motor->close(motor);

    return 0;
}

static ssize_t file_read(struct file* f, char* buf, size_t bytes, loff_t* offset)
{
    Motor* motor = (Motor*) f->private_data;
    struct timeval tv;
    long us;

    do_gettimeofday(& tv);
    us = (1000000 * tv.tv_sec) + tv.tv_usec;

    sprintf(buf, "%ld\t%d\t%d\n", us, motor->get_counter(motor), motor->get_speed(motor));

    return strlen(buf);
}

static ssize_t file_write(struct file* f, const char* buf, size_t bytes, loff_t* offset)
{
    Motor* motor = (Motor*) f->private_data;
    do_command(motor, buf, bytes);
    return bytes;
}

static struct file_operations fops = {
    .read = file_read,
    .write = file_write,
    .open = file_open,
    .release = file_release,
    .owner = THIS_MODULE,
};

    /*
     *  Config params
     */

static int pwm0[5];
int pwm0_count = 5;
module_param_array(pwm0, int, & pwm0_count, S_IRUGO);

static int pwm1[5];
int pwm1_count = 5;
module_param_array(pwm1, int, & pwm1_count, S_IRUGO);

static int step0[5];
int step0_count = 5;
module_param_array(step0, int, & step0_count, S_IRUGO);

static int step1[5];
int step1_count = 5;
module_param_array(step1, int, & step1_count, S_IRUGO);

static int major;
module_param(major, int, S_IRUGO);

static bool set_pwm_desc(PwmDesc* desc, int* args)
{
    desc->pwm_chan   = *args++;
    desc->gpio_dir   = *args++;
    desc->gpio_en    = *args++;
    desc->gpio_clock = *args++;
    desc->gpio_sense = *args;

    printk("pwm chan:%d dir:%d en:%d clock:%d sense:%d\n", 
            desc->pwm_chan,
            desc->gpio_dir,
            desc->gpio_en,
            desc->gpio_clock,
            desc->gpio_sense);

    return desc->gpio_dir; // must be set
}

static bool set_stepper_desc(StepperDesc* desc, int* args)
{
    desc->gpio_a = *args++;
    desc->gpio_nega = *args++;
    desc->gpio_b = *args++;
    desc->gpio_negb = *args++;
    desc->gpio_en = *args;

    printk("step a=%d,nega=%d b=%d negb=%d en=%d\n", 
            desc->gpio_a, desc->gpio_nega, 
            desc->gpio_b, desc->gpio_negb,
            desc->gpio_en);

    return desc->gpio_a && desc->gpio_nega 
            && desc->gpio_b && desc->gpio_negb 
            && desc->gpio_en; // must all be set
}

    /*
     *  Module
     */

static struct cdev cdev;

static int read_proc(char* buf, char** start, off_t offset, int count, int* eof, void* data)
{
    Motor* motor = (Motor*) data;
    struct timeval tv;
    long us;
    //printk(KERN_INFO "motor_read\n");

    //sprintf(buf, "%ld\t%d\t%d\n", jiffies, motor->get_counter(motor), motor->get_speed(motor));
    do_gettimeofday(& tv);
    us = (1000000 * tv.tv_sec) + tv.tv_usec;

    sprintf(buf, "%ld\t%d\t%d\n", us, motor->get_counter(motor), motor->get_speed(motor));

    return strlen(buf);
}

static int __init do_init(void)
{
    PwmDesc pdesc;
    StepperDesc sdesc;
    int device_index = 0;
    int dev;
    int err;

    printk(KERN_INFO "hello motor driver\n");

    if (!major)
    {
        err = alloc_chrdev_region(& dev, 0, 1, "motor");
        if (err < 0)
        {
            printk("Can't allocate major number : %d\n", err);
            return err;
        }
        printk("motor major=%d minor=%d\n", MAJOR(dev), MINOR(dev));
    }
    else
    {
        dev = MKDEV(major, 0);
        printk("motor major=%d minor=%d\n", MAJOR(dev), MINOR(dev));
    }

    if (set_pwm_desc(& pdesc, pwm0))
    {
        pwm_motor_init(& motors[device_index++], "pwm0", & pdesc);
        //  TODO : mknod the device if it does not exist? 
    }
    if (set_pwm_desc(& pdesc, pwm1))
    {
        pwm_motor_init(& motors[device_index++], "pwm1", & pdesc);
        //  TODO : mknod the device if it does not exist? 
    }
    if (set_stepper_desc(& sdesc, step0))
    {
        stepper_init(& motors[device_index++], "step0", & sdesc);
        //  TODO : mknod the device if it does not exist? 
    }
    if (set_stepper_desc(& sdesc, step1))
    {
        stepper_init(& motors[device_index++], "step1", & sdesc);
        //  TODO : mknod the device if it does not exist? 
    }

    //  Register the char device driver
    cdev_init(& cdev, & fops);
    cdev.owner = THIS_MODULE;
    cdev.ops = & fops;
    err = cdev_add(& cdev, dev, device_index);
    if (err < 0)
    {
        printk("Error registering driver : %d\n", err);
        return err;
    }
    printk("registered minor=0..%d\n", device_index-1);

    create_proc_read_entry("motor", 0, NULL, read_proc, & motors[0]);

    return 0;
}

static void __exit do_cleanup(void)
{
    printk(KERN_INFO "cleanup motor driver\n");

    cdev_del(& cdev);

    remove_proc_entry("motor", NULL);
}

module_init(do_init);
module_exit(do_cleanup);

// FIN
