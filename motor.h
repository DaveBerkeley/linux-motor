
#include "pwm.h"
#include "stepper.h"

extern irqreturn_t on_irq(int irq, void* dev_id);

//  Common API. I miss C++

#define MAX_IRQS    2

typedef struct Motor
{
    bool            is_open;        //  true if open
    int             irqs[MAX_IRQS]; //  irq numbers for input pin
    const char*     name;
    int             speed;

    union {
        PwmMotor        pwm;
        StepperMotor    stepper;
    }   m;

    int (*open)(struct Motor* motor);
    void (*close)(struct Motor* motor);
    void (*move_to)(struct Motor* motor, int posn);
    void (*set_speed)(struct Motor* motor , int speed);
    int (*get_speed)(struct Motor* motor);
    void (*set_counter)(struct Motor* motor, int count);
    int (*get_counter)(struct Motor* motor);
    bool (*config)(struct Motor* motor, const char* str);

    bool (*on_irq)(struct Motor* motor, int index);
}   Motor;

int pwm_motor_init(Motor* m, const char* name, const void* init);
int stepper_init(struct Motor* m, const char* name, const void* init);

    /*
     *  GPIO Utils
     */

#define GPIO_FN     0
#define ALT_FN_1    1
#define ALT_FN_2    2
#define ALT_FN_3    3

void GPIO_mode(int gpio, int mode, bool out);

    /*
     *  Counter Timer hardware
     */

#ifdef OSCR
#undef OSCR
#endif

// WARNING: Only works for timers > 3.
#define OMCR(n) (*(unsigned int*)io_p2v(0x40a000C0 + ((n-4)*4)))
#define OSMR(n) (*(unsigned int*)io_p2v(0x40a00080 + ((n-4)*4)))
#define OSCR(n) (*(unsigned int*)io_p2v(0x40a00040 + ((n-4)*4)))
#define TIMER_IRQ PXA_IRQ(7)

int timer_open(int mode);
void timer_close(int timer);
void timer_enable(int timer);
void timer_period(int timer, int period);
bool timer_on_irq(int timer);

// FIN
