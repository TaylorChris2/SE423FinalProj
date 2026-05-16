#include <math.h>
#include "F28x_Project.h"
#include "MatrixMath.h"
#include "AprilTag.h"

#define PI 3.14159265f

float OPENMV_TO_FEET = 1.0f;

// Camera mount relative to robot center in ft
float camera_x_robot = 7.5f / 12.0f;
float camera_y_robot = 0.0f;
float camera_theta_robot = 0.0f;

// AprilTag-estimated robot pose
float april_robot_x = 0.0f;
float april_robot_y = 0.0f;
float april_robot_theta = 0.0f;

void April_Init(void)
{
    NewAprilRobotPose = 0;

    april_robot_x = 0.0f;
    april_robot_y = 0.0f;
    april_robot_theta = 0.0f;
}


void April_ComputeRobotPose2D_v2(
    float tagid,
    float tagx,
    float tagy,
    float tagz,
    float tagthetax,
    float tagthetay,
    float tagthetaz
) {
    
    //JCCA current world location variables as measured
    float current_tag_world_x = 0.0f;
    float current_tag_world_y = 0.0f;
    float current_tag_world_theta = 0.0f;
    // Flag indicating whether the detected tag is recognized
    uint16_t known_tag = 1;

    //JCCA This is a switch case to determine the tagid and the location that this tagid corresponds to. Only 1 tag was tested for now
    switch ((int)tagid) {

    case 0:
        current_tag_world_x = 2.0f;
        current_tag_world_y = 0.0f;
        current_tag_world_theta = 0.0f;
        break;
        

    default:
        known_tag = 0;
        break;
    }

     //JCCA Compute distance from camera to tag.Uses x and z coordinates from camera frame.
    float r = sqrt(tagx*tagx +tagz*tagz);
    //JCCA total orientation between world frame and camera observation. Converts tag rotation from degrees to radians.
    float t_tot = current_tag_world_theta + PI - tagthetaz*PI/180.0;
    //JCCA Relative displacement from robot/camera to tag. Coordinate frame conversion: camera z -> world x camera x -> world y
    float d_xa = tagz;
    float d_ya = tagx;
    
    //JCCA Compute robot world position using rotation and translation transforms.
    april_robot_x = -d_xa * cosf(t_tot) + d_ya * sinf(t_tot) + current_tag_world_x;
    april_robot_y = -d_xa * sinf(t_tot) - d_ya * cosf(t_tot) + current_tag_world_y;
    //JCCA Compute robot heading angle. atan2 gives orientation toward the tag.
        april_robot_theta = atan2(current_tag_world_y - april_robot_y, current_tag_world_x - april_robot_x);

}
