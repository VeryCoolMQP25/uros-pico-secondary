#ifndef TUNABLES_H
#define TUNABLES_H

#define PID_DT_V_KP	1.0
#define PID_DT_V_KI	0.1
#define PID_DT_V_KD	0.05

#define PID_DT_KP	10.0
#define PID_DT_KI	0.1
#define PID_DT_KD	0.05
#define PID_DT_TOL	0.05

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
#define MOTOR_POWER_MAX	40

#endif
