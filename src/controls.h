#ifndef CONTROLS_H
#define CONTROLS_H

typedef enum {
	cs_disconnected,
	cs_connected,
	cs_down
} CommState;

typedef enum {
	dm_raw,
	dm_twist,
	dm_halt
} DriveMode;

#endif
