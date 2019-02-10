#ifndef LightingControl_h
#define LightingControl_h
#include "arduino.h"

#define TREAD 270
#define RADIUS 70
#define GEAR_RATIO 17600
#define VPGAIN_r 0.125
#define VDGAIN_r 0.0
#define VIGAIN_r 0.0
#define VPGAIN_l 0.125
#define VDGAIN_l 0.0
#define VIGAIN_l 0.0

#define PULSE_TO_MM 0.012494970781323	                      //1pulse[mm]
#define SPEED_RIGHT_REF 150                             //target speed[mm/s]
#define SPEED_LEFT_REF 150
#define PWM_MAX 255


class parameters{
	public:
		parameters();
	private:
};

#endif
