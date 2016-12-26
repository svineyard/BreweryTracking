/*******************************************************************************
* balance.c
*
* This program implements a discrete time controller designed and simulated
* initially in Matlab with the objective of balancing our MIP. 
*
*******************************************************************************/

#include "balance.h"

// Global variables
float angle_accel = 0;
float angle_gyro = 0;
float angle_both = 0;
const float dt = 1.0/(float)SAMPLE_RATE;
float lp_angle_accel, hp_angle_gyro, both_accel_gyro = 0;
int arm_state;
int first_iteration = 1;

// Global structs 
d_filter low_pass, high_pass, D1, D2;
state sys_state;
setpoint_t setpoint;
imu_data_t data;

// low and high pass filter coefficients for transfer function
float num_low[] = {0.04766, 0.04766};
float den_low[] = {1, -0.9047};
float num_high[] = {0.9524, -0.9524};
float den_high[] = {1, -0.9048};

// variable gain coefficients for D1 and D2
const float K_D1 = 1;
const float K_D2 = 1;

//D1 and D2 controller coefficients for transfer function
float D1_num[] = {-51.19, 86.68,-36.69};
float D1_den[] = {1, -0.4895, -0.5105};
float D2_num[] = {0.1018,-0.1008};
float D2_den[] = {1 -0.621};

/*******************************************************************************
* int main
*
* Initializes IMU, D1 and D2 controllers, and sets up controller interrupt 
* function.
*
*******************************************************************************/
int main(){

    set_cpu_frequency(FREQ_1000MHZ);
    
    // always initialize cape library first
    if(initialize_cape()){
        printf("ERROR: Failed to initialize_cape\n");
        return -1;
    }

    set_led(RED,1);
    set_led(GREEN,0);
    set_state(UNINITIALIZED);

    // Start controller as disarmed 
    arm_state = DISARMED;
    
    // D1 controller for theta
    D1 = new_filter(2, DT, D1_num, D1_den);
    D1.gain = D1_GAIN;
    set_saturation(&D1, -1.0, 1.0);
    set_soft_start(&D1, 0.6);
    
    // D2 controller for phi 
    D2 = new_filter(1, DT, D2_num, D2_den);
    set_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);

    // start with default config and then modify sample rate to 200 Hz
    imu_config_t conf = get_default_imu_config();
    conf.dmp_sample_rate = 200;
    conf.orientation = ORIENTATION_Y_UP;

    // set up imu for dmp interrupt operation
    if(initialize_imu_dmp(&data, conf)){
        printf("initialize_imu_failed\n");
        return -1;
    }

    // label for angle estimate data
    print_header();

    // Set up balance controller as IMU interrupt 
    set_imu_interrupt_func(&balance_controller);

    // Check to make sure sys_state.phi has a wheel position estimate
    // before proceeding to setting the outer loop D2 thread
     while(sys_state.phi==0 && get_state()!=EXITING) usleep(1000);

    // Set up separate thread for outer loop D2 
    pthread_t D2_thread;
    pthread_create(&D2_thread, NULL, wheel_position_controller, (void*) NULL);

    // Initialize state to running 
    set_state(RUNNING);

    // Wait while controller function is called by interrupt
    while (get_state()!=EXITING) {
        usleep(10000);
    }

    // shut things down and exit 
    power_off_imu();
    cleanup_cape(); 
    set_cpu_frequency(FREQ_ONDEMAND);
    return 0;
}

/*******************************************************************************
 * balance_controller
 * 
 * Whenever the IMU has new data, this interrupt controller function D1 will be 
 * called to balance the MIP.
 * *******************************************************************************/
int balance_controller(){

    float duty_L, duty_R;
    static int sat_count = 0;

    low_pass = new_filter(1,dt, num_low, den_low );
    high_pass = new_filter(1,dt, num_high, den_high );

    printf("\r");
    printf(" ");
    
    // Average body angle estimations from low pass filtering of accel and high pass
    // filtering of gyro.
    angle_accel = atan2(-1*(data.accel[2]), data.accel[1]);
    lp_angle_accel = next_time_step(&low_pass, angle_accel);
    
    angle_gyro = angle_gyro + data.gyro[0]*DEG_TO_RAD*DT;
    hp_angle_gyro = next_time_step(&high_pass, angle_gyro);
        
    sys_state.theta = lp_angle_accel + hp_angle_gyro; //- 30*DEG_TO_RAD;

    // Average wheel position estimations from both encoders
    sys_state.wheelAngle_R = (get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI)\
                            /(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
    sys_state.wheelAngle_L = (get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI)\
                            /(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
    sys_state.phi = (sys_state.wheelAngle_L + sys_state.wheelAngle_R)/2 + \
                    sys_state.theta;

    printf("      theta: %10.2f rads             phi: %10.2f rads",
            sys_state.theta, sys_state.phi);
   
    // disable motors if state is set to exiting 
    if(get_state() == EXITING){
        disable_motors();
        return 0;
    }

    // exit if controller has been disarmed 
    if(arm_state == DISARMED){
        return 0;
    }
    
    // check if MIP has fell past our max tipping angle and disarm 
    // the controller.
    if(fabs(sys_state.theta) > 0.8 && arm_state == ARMED){
        disarm_controller();
        return 0;
    }

    // check if MIP is back within a starting angle range and arm 
    // the controller.
    if(fabs(sys_state.theta) < 0.3 && arm_state == DISARMED){
        arm_controller();
        return 0;
    }
        
    // D1 controller for inner loop of body angle theta 
    sys_state.d1_u = next_time_step(&D1,setpoint.theta - sys_state.theta);

    // check for saturation to prevent stalling of motors. 
    if(check_saturation(&D1)) sat_count++;
    else sat_count = 0; 
    //Saturation past 1 second results in disarming the controller 
    if(sat_count > (SAMPLE_RATE*0.5)){
        disarm_controller();
        sat_count = 0;
        return 0;
    }
    
    // Finally, in all its glory, we can send a duty cycle to our motors
    duty_L = sys_state.d1_u;
    duty_R = sys_state.d1_u;  
    set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * duty_L); 
    set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * duty_R); 

    return 0;
}

/*******************************************************************************
 * wheel_position_controller
 * 
 * Update reference theta for D1 at 20 Hz.
 ********************************************************************************/
void* wheel_position_controller(void* ptr){
    // For 20 Hz rate of execution
    int D2_hz = 20;
    int wait_us = 1000000/D2_hz;
    usleep(wait_us);

    if(first_iteration == 1) {
        setpoint.phi = sys_state.phi;
        first_iteration = 0;
    }
    sys_state.d2_u = next_time_step(&D2,setpoint.phi-sys_state.phi);
    setpoint.theta = sys_state.d2_u;
    return 0;
}

/*******************************************************************************
 * print_header
 * 
 * Print label for accel data printed to console.   
 ********************************************************************************/
void print_header(){

    printf(" ");
    
    printf("Body Angle Estimation theta (rad) | Average Wheel Angle " 
                "Estimation phi (rads");
   
    printf("\n\n");
}

/*******************************************************************************
*  clear_controller
*  
* Sets previous inputs and outputs for difference equation of controller
* to zero
*******************************************************************************/
int clear_controller(){
    setpoint.theta = 0.0;
    setpoint.phi   = 0.0;
    clear_controller(&D1);
    clear_controller(&D2);
    set_motor_all(0);
    return 0;
}

/*******************************************************************************
* disarm_controller
*
* Set arm state to disarmed 
*******************************************************************************/
int disarm_controller(){
    arm_state = DISARMED;
    disable_motors();
    set_led(GREEN,0);
    set_led(RED,1);
    return 0;
}

/*******************************************************************************
* arm_controller
*
* Initializes encoder positions to zero and clears previous inputs and outputs
*******************************************************************************/
int arm_controller(){
    arm_state = ARMED;
    clear_controller();
    set_encoder_pos(ENCODER_CHANNEL_L,0);
    set_encoder_pos(ENCODER_CHANNEL_R,0); 
    enable_motors();
    set_led(RED,0);
    set_led(GREEN,1);
    return 0;
}
