
    /*
     *  Stepper Motor
     */

typedef struct 
{
    int     gpio_a;
    int     gpio_nega;
    int     gpio_b;
    int     gpio_negb;
    int     gpio_en;
}   StepperDesc;

enum StepperState
{
    STOP = 0,
    ACCEL,
    MOVE,
    DECEL,
};

typedef struct 
{
    StepperDesc desc;

    int     state;
    int     accel;
    int     vmin;
    int     vmax;

    //  timer channel (4..11)
    int     timer;
    //
    int     position;
    int     target;
    int     req_v;
    int     start;
    int     mid_point;
    int     decel_point;
    int     v;
}   StepperMotor;

// FIN
