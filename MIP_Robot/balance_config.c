/*******************************************************************************
 * balance_config.c
 *
 *******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "balance.h"

/*******************************************************************************
 * new_filter
 *
 * Generate new filter based on transfer function constants and dt
 *
 *******************************************************************************/

d_filter new_filter(int order, float dt,float* num,float* den){
    d_filter filter;

    filter.gain = 1;
    filter.newest_input = 0;
    filter.newest_output = 0;
    filter.numerator = create_vector_from_array(order+1, num);
    filter.denominator = create_vector_from_array(order+1, den);
    filter.in_buf = create_ring_buf(order+1);
    filter.out_buf = create_ring_buf(order+1);
    filter.step = 0;
    filter.saturation = 0;
    filter.saturation_check = 0;
    filter.saturation_min = 0;
    filter.saturation_max = 0;
    filter.soft_start = 0;
    filter.soft_start_steps = 0;
    return filter;
}

/*******************************************************************************
* clear_filter
*
* clears all inputs and outputs to zero
*******************************************************************************/
int clear_filter(d_filter* filter){
    reset_ring_buf(&filter->in_buf);
    reset_ring_buf(&filter->out_buf);
    filter->newest_input = 0;
    filter->newest_output = 0;
    filter->step = 0;
    return 0;
}

/*******************************************************************************
 * next_time_step
 *
 * Return a new output one dt step in time for given input. Utilizes ring buffer
 * data structure to store previous inputs 
 *
 *******************************************************************************/

float next_time_step(d_filter* filter, float new_input){

    int i = 0;

    //implement ring buffer data structure to store new inputs
    insert_new_ring_buf_value(&filter->in_buf, new_input);
    filter->newest_input = new_input;

    // evaluate difference equation
    float new_output = 0;
    float input_i, output_i;
    for(i=0; i<=1; i++){
        input_i = get_ring_buf_value(&filter->in_buf,i);
        new_output += filter->gain * filter->numerator.data[i] * input_i;
    }
    for(i=1; i<=1; i++){
        output_i = get_ring_buf_value(&filter->out_buf,i-1);
        new_output -= filter->denominator.data[i] * output_i;
        }

    //scale output by denominator 
    new_output = new_output/filter->denominator.data[0];

    // saturation 
    if(filter->saturation){
        if(new_output > filter->saturation_max){
            new_output = filter->saturation_max;
            filter->saturation_check=1;
        }
        else if(new_output < filter->saturation_min){
            new_output = filter->saturation_min;
            filter->saturation_check=1;
        }
        else{
            filter->saturation_check=0;
        }
    }

    // soft start limits
    if(filter->soft_start && filter->step < filter->soft_start_steps){
        float max =filter->saturation_max*(filter->step/filter->soft_start_steps);
        float min =filter->saturation_min*(filter->step/filter->soft_start_steps);
        if(new_output > max) new_output = max;
        if(new_output < min) new_output = min;
    }

    // Write newest output to our filter struct and ring buffer 
    filter->newest_output = new_output;
    insert_new_ring_buf_value(&filter->out_buf, new_output);
    //increment step count 
    filter->step++;
    return new_output;
}

/*******************************************************************************
* set_soft_start
* 
* Slowly increases saturation limits at initialization of controller for 
* balancing. Avoids jerky initial motions.
*******************************************************************************/
int set_soft_start(d_filter* filter, float seconds){
    filter->soft_start = 1;
    filter->soft_start_steps = seconds/filter->dt;
    return 0;
}

/*******************************************************************************
* check_saturation
*
* Returns a 1 if the controller is already saturated.
*******************************************************************************/
int check_saturation(d_filter* filter){
    return filter->saturation_check;
    return 0;
}

/*******************************************************************************
* set_saturation
*
* Sets a cap at both min and max for output duty cycle to our motors to prevent
* stalling.
*******************************************************************************/
int set_saturation(d_filter* filter, float min, float max){
    filter->saturation = 1;
    filter->saturation_min = min;
    filter->saturation_max = max;
    return 0;
}
