#ifndef TUNABLES_H
#define TUNABLES_H

// distance between front wheels in meters
#define WHEELBASE_M 0.51

#define PID_DT_V_KP	0.62
#define PID_DT_V_KI	0.5
#define PID_DT_V_KD	0.001
#define PID_DT_TOL	0.06
#define PID_DT_KI_CAP	3.0
//old:
// #define PID_DT_V_KP	0.8
// #define PID_DT_V_KI	0.95
// #define PID_DT_V_KD	0.001

#define PID_LFT_KP	10.0
#define PID_LFT_KI	0.1
#define PID_LFT_KD	0.05
#define PID_LFT_TOL	0.05

/*
+-----------+-------+-------+------------------+------------------+
| dist (cm) | L     | R     | L/dist           | R/dist           |
+-----------+-------+-------+------------------+------------------+
| -20       | -5351 | -5457 | 267.55           | 272.85           |
| 30        | 7679  | 7719  | 255.966666666667 | 257.3            |
| 40        | 10239 | 10358 | 255.975          | 258.95           |
| 45        | 11632 | 11310 | 258.488888888889 | 251.333333333333 |
| 10        | 2378  | 2446  | 237.8            | 244.6            |
| 10        | 2549  | 2605  | 254.9            | 260.5            |
+-----------+-------+-------+------------------+------------------+
*/

#define DT_ENCODER_PPM	25635
#define MOTOR_POWER_MAX	100

#endif
