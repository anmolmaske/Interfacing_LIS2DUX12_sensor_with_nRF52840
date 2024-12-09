#include "switch_monitoring.h"

LOG_MODULE_REGISTER(SWITCH);

uint8_t state_mode = 2;
uint8_t axis_mode = 1;

double x_axis_angle;
double y_axis_angle;
double z_axis_angle;

int low_angle_threshold = 30;
int high_angle_threshold = 30;
int delay_after_rail_switching = 2000;

/*Compare and print threshold crossed with respect to angle selected */
void switch_monitoring_data(void)
{   
    fetch_lis_sensor_values();

    /* X-Axis is selected*/
    if(axis_mode == 1)
    { 
        x_axis_angle = low_gx; 
       
        if(x_axis_angle >= high_angle_threshold || x_axis_angle <= (0 - low_angle_threshold) )
        {   
            if(x_axis_angle >= high_angle_threshold){
                state_mode = 1;
                LOG_INF("Threshold crossed on the +ve X-axis: %.2f, State Changed to: %d ",(double)x_axis_angle, state_mode);
            } else {
                state_mode = 0;
                LOG_INF("Threshold crossed on the -ve X-axis: %.2f, State Changed to: %d ",(double)x_axis_angle, state_mode);
            }
        }
        else 
        {
            state_mode = 2;
             LOG_INF("X-axis: %.2f, State Changed to: %d ",(double)x_axis_angle, state_mode);
        }
    }

    /* Y-Axis is selected*/
    else if(axis_mode == 2)
    {
        y_axis_angle = low_gy;

        if(y_axis_angle >=high_angle_threshold || y_axis_angle <= (0 - low_angle_threshold) )
        {
            if(y_axis_angle >=high_angle_threshold){
                state_mode = 1;
                LOG_INF("Threshold crossed on the +ve Y-axis: %.2f, State Changed to: %d ",(double)y_axis_angle, state_mode);
            } else {
                state_mode = 0;
                LOG_INF("Threshold crossed on the -ve Y-axis: %.2f, State Changed to: %d ",(double)y_axis_angle, state_mode);
            }
        }
        else
        {
            state_mode = 2;
            LOG_INF("Y-axis: %.2f, State Changed to: %d ",(double)y_axis_angle, state_mode);
        }
    }
    
    /* Z-Axis is selected*/
   else if(axis_mode == 3)
   {
        z_axis_angle = low_gz;

        if(z_axis_angle >= high_angle_threshold || z_axis_angle <= (0 - low_angle_threshold))
        {
            if(z_axis_angle >= high_angle_threshold) {
                state_mode = 1;
                LOG_INF("Threshold crossed on the +ve Z-axis: %.2f, State Changed to: %d ",(double)z_axis_angle, state_mode);
            } else {
                state_mode = 0;
                LOG_INF("Threshold crossed on the -ve Z-axis: %.2f, State Changed to: %d ",(double)z_axis_angle, state_mode);
            }
        }
        else
        {
            state_mode = 2;
            LOG_INF("Z-axis: %.2f, State Changed to: %d ",(double)z_axis_angle, state_mode);
        }
   }
}

/*Read Switch Monitor and print on RTT*/
void read_switch_monitoring(void){

    if (motion_detected) {

        /* Read Interrupt Source */
        lis2dux12_all_sources_t int_src;
        if (lis2dux12_all_sources_get(&lis_ctx, &int_src) == 0) {

            if (int_src.wake_up_x) {
                LOG_INF("Wake-up interrupt detected on X-Axis!\n");
            }
            if (int_src.wake_up_y) {
                LOG_INF("Wake-up interrupt detected on Y-Axis!\n");
            }
            if (int_src.wake_up_z) {
                LOG_INF("Wake-up interrupt detected on Z-Axis!\n");
            }
        } else {
            LOG_INF("Error: Failed to read interrupt source register.\n");
        }

        LOG_INF("Motion Detected! Now you can read Switch State Data...\n");

        /* Print data on RTT */
        switch_monitoring_data();

        motion_detected = false;  // Clear the flag
    }
}
