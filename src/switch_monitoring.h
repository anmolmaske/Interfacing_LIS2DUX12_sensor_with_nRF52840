#ifndef SWT_MONITORING_H
#define SWT_MONITORING_H

#include <zephyr/logging/log.h>
#include "lis2dux12_reg.h" // Include your sensor driver header

extern stmdev_ctx_t lis_ctx;

extern bool motion_detected;

extern float low_gx;
extern float low_gy;
extern float low_gz;

/* Function declarations */
void read_switch_monitoring(void);

#endif // SWT_MONITORING_H