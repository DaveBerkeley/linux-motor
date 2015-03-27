

typedef struct 
{
    unsigned int*   cr;
    unsigned int*   dcr;
    unsigned int*   pcr;
    int             gpio;
    int             ck_mask;
}   PWM;

//  Motor data structure

typedef struct 
{
    //  Chan/pin definitions for a single motor.
    int     pwm_chan;   //  0|1
    //  GPIO number, eg. 101
    int     gpio_dir;   //  motor direction - opposite side of H-bridge to PWM
    int     gpio_en;    //  tri-state the H-bridge
    int     gpio_clock; //  clock input from position encoder
    int     gpio_sense; //  dir input from position encoder
}   PwmDesc;

typedef struct 
{
    int             position;   //  position encoder
    //  Internal
    const PWM*      pwm;
    PwmDesc         md;         //  initial config params
    int             timer;
    //  pwm state
    int             period;
    int             mark;
}   PwmMotor;

#define PWMREG(n)   (unsigned int*)io_p2v(n)

static inline void pwm_set(const char* name, unsigned int* reg, int value)
{
    *reg = value;
}

static inline void pwm_cr(const PWM* pwm, int value)
{
    pwm_set("pwm_cr: ", pwm->cr, value);
}

static inline void pwm_dcr(const PWM* pwm, int value)
{
    pwm_set("pwm_dcr:", pwm->dcr, value);
}

static inline void pwm_pcr(const PWM* pwm, int value)
{
    pwm_set("pwm_pcr:", pwm->pcr, value);
}

//  FIN
