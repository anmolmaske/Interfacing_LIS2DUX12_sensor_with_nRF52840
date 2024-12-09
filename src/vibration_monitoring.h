#ifndef VIB_MONITORING_H
#define VIB_MONITORING_H

#include <zephyr/logging/log.h>
#include "lis2dux12_reg.h" // Include your sensor driver header

extern stmdev_ctx_t lis_ctx;

extern bool motion_detected;

/* Function declarations */
void read_vibration_monitoring(void);

#endif // VIB_MONITORING_H
