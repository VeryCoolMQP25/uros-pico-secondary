#ifndef TUNABLES_H
#define TUNABLES_H

// distance between front wheels in meters
#define WHEELBASE_M 0.51

#define PID_DT_V_KP	0.65
#define PID_DT_V_KI	1.55
#define PID_DT_V_KD	0.003
#define PID_DT_TOL	0.001
#define PID_DT_KI_CAP	0.35
//old:
// #define PID_DT_V_KP	0.8
// #define PID_DT_V_KI	0.95
// #define PID_DT_V_KD	0.001

#define PID_LFT_KP	10.0
#define PID_LFT_KI	0.1
#define PID_LFT_KD	0.05
#define PID_LFT_TOL	0.05

/* distances averaged from measurements
+---------------+-----------+-----------+
| Distance (cm) | Encoder L | Encoder R |
+---------------+-----------+-----------+
| 5             | 6686      | 6787      |
| 10            | 14483     | 14614     |
| 15            | 22582     | 22491     |
| 32            | 48353     | 47352     |
| 64            | 98647     | 98176     |
| 109           | 167258    | 167718    |
+---------------+-----------+-----------+
*/
#define DT_ENCODER_PPM	147904
#define MOTOR_POWER_MAX	100

#endif

// [13235] DEBUG: P:0.019370, I:0.019791, D:-0.393729
// tgt: 0.267728, act: 0.240057, err: 0.027671 | out: -35
