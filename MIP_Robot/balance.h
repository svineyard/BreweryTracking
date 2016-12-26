
/*******************************************************************************
 * balance.h
 *
 * This is header file that declares the structs and defines all
 * the constants for balance.c
 *
 *****************************************************************************/

#include <roboticscape.h>
#include <roboticscape-usefulincludes.h>

#ifndef BALANCE
#define BALANCE

// armed and disarmed states
#define ARMED                                  1
#define DISARMED                               0

//sample rate, time constant, and dt
#define DT                                     0.01
#define SAMPLE_RATE                            100
#define TIME_CONSTANT                          0.1

// Structural properties of eduMiP
#define CAPE_MOUNT_ANGLE	        	0.40
#define GEARBOX 				35.577
#define ENCODER_RES				60
#define WHEEL_RADIUS_M		            	0.034
#define TRACK_WIDTH_M			        0.035

// outer loop controller 20 hz
#define THETA_REF_MAX		        	0.4

// electrical hookups
#define MOTOR_CHANNEL_L 			3
#define MOTOR_CHANNEL_R	    	        	2
#define MOTOR_POLARITY_L        		1
#define MOTOR_POLARITY_R	        	-1
#define ENCODER_CHANNEL_L	        	3
#define ENCODER_CHANNEL_R	        	2
#define ENCODER_POLARITY_L      		1
#define ENCODER_POLARITY_R	        	-1


/*******************************************************************************
 * setpoint
 *
 * Setpoint for balance controller 
 *
 *****************************************************************************/

 typedef struct setpoint_t{
     float theta;
     float phi;
 }setpoint_t;

/*******************************************************************************
 * state
 *
 * This struct contains variables used by the controller function for system
 * state updating. 
 *
 *****************************************************************************/

typedef struct state{
    float wheelAngle_R;  // wheel angle for right motor 
    float wheelAngle_L;  // wheel angle for left motor
    float theta;         // body angle in radians 
    float phi;           // average wheel angle
    float volt;         // battery voltage 
    float d1_u;          // output of controller D1 to motors
    float d2_u;          // output of controller D2 to motors 
} state;


/*******************************************************************************
 * d_filter
 *
 * This is a struct type definition that delclares variables for discrete
 * filtering 
 *
 *****************************************************************************/

typedef struct d_filter{
    //define transfer function constants
    int order;
    float dt;
    float gain;
    vector_t numerator;
    vector_t denominator;
    //dynamically allocated ring buffers
    ring_buf_t in_buf;
    ring_buf_t out_buf;
    //newest input and output
    float newest_input;
    float newest_output;
    // saturation 
    int saturation;
    int saturation_check;
    float saturation_min;
    float saturation_max;
    // soft start
    int soft_start;
    float soft_start_steps;
    //other
    uint64_t step;  // steps since last reset
    int initalized;
} d_filter;

// declare all functions being used
d_filter new_filter(int order, float dt, float* num, float* den);
float next_time_step(d_filter* filter, float new_input);
int balance_controller();
void* wheel_position_controller(void* ptr);
int disarm_controller();
int arm_controller();
int set_saturation(d_filter* filter, float max, float min);
int set_soft_start(d_filter* filter, float seconds);
void print_header();
int check_saturation(d_filter* filter);
int clear_controller();
int clear_filter(d_filter* filter);


#endif //BALANCE

