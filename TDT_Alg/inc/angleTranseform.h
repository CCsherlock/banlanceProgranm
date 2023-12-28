#ifndef _ANGLE_TRANSEFORM_H_
#define _ANGLE_TRANSEFORM_H_
#include "my_math.h"
float standThetaCal(float thetaNow, float targetTheta);
float jumpThetaCal(float thetaNow, float targetTheta, float jumpAngle, int8_t dirction);
#endif