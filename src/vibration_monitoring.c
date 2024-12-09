#include "vibration_monitoring.h"

LOG_MODULE_REGISTER(VIBRATION);

/*Read Vibration Monitor and print on RTT*/
void read_vibration_monitoring(){

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

            LOG_INF("Vibrations Detected! Now you can read sound...\n");

            motion_detected = false;  // Clear the flag
        }
}

