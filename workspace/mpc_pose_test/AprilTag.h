#ifndef APRILTAG_H_
#define APRILTAG_H_

#include <stdint.h>

extern float OPENMV_TO_FEET;

extern float camera_x_robot;
extern float camera_y_robot;
extern float camera_theta_robot;

extern float april_robot_x;
extern float april_robot_y;
extern float april_robot_theta;

extern uint16_t NewAprilRobotPose;

void April_Init(void);

void April_Make2DTransform(float T[3][3], float x, float y, float theta);

void April_ComputeRobotPose2D(
    float tagid,
    float tagx,
    float tagy,
    float tagz,
    float tagthetax,
    float tagthetay,
    float tagthetaz
);

#endif
