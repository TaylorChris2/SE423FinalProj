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

uint16_t NewAprilRobotPose = 0;

float T_world_tag_2D[3][3];
float T_camera_tag_2D[3][3];
float T_tag_camera_2D[3][3];
float T_robot_camera_2D[3][3];
float T_camera_robot_2D[3][3];
float T_world_camera_2D[3][3];
float T_world_robot_2D[3][3];


void April_Make2DTransform(float T[3][3], float x, float y, float theta)
{
    T[0][0] = cosf(theta);     T[0][1] = -sinf(theta);    T[0][2] = x;
    T[1][0] = sinf(theta);     T[1][1] = cosf(theta);     T[1][2] = y;
    T[2][0] = 0.0f;  T[2][1] = 0.0f;  T[2][2] = 1.0f;
}


void April_Init(void)
{
    NewAprilRobotPose = 0;

    april_robot_x = 0.0f;
    april_robot_y = 0.0f;
    april_robot_theta = 0.0f;
}


// void April_ComputeRobotPose2D(
//     float tagid,
//     float tagx,
//     float tagy,
//     float tagz,
//     float tagthetax,
//     float tagthetay,
//     float tagthetaz
// )
// {

//     float tag_forward_camera;
//     float tag_left_camera;
//     float tag_theta_camera;

//     float current_tag_world_x = 0.0f;
//     float current_tag_world_y = 0.0f;
//     float current_tag_world_theta = 0.0f;

//     uint16_t known_tag = 1;

//     // No tag detected
//     if (tagid < 0.0f) {
//         NewAprilRobotPose = 0;
//         return;
//     }

//     // Pick known world pose based on detected tag ID
//     switch ((int)tagid) {

//         case 0:
//             // Example: tag 0 is 2 ft in front of world origin
//             current_tag_world_x = 2.0f;
//             current_tag_world_y = 0.0f;
//             current_tag_world_theta = 0.0f;
//             break;

//         default:
//             known_tag = 0;
//             break;
//     }

//     if (known_tag == 0) {
//         NewAprilRobotPose = 0;
//         return;
//     }

//     // OpenMV camera frame:
//     //   tagz = forward
//     //   tagx = right
//     //
//     // Robot/world 2D frame:
//     //   +X = forward
//     //   +Y = left
//     tag_forward_camera = tagz * OPENMV_TO_FEET;
//     tag_left_camera = -tagx * OPENMV_TO_FEET;


//     // Known tag pose in world
//     April_Make2DTransform(
//         T_world_tag_2D,
//         current_tag_world_x,
//         current_tag_world_y,
//         current_tag_world_theta
//     );

//     // Detected tag pose relative to camera
//     April_Make2DTransform(
//         T_camera_tag_2D,
//         tag_forward_camera,
//         tag_left_camera,
//         tag_theta_camera
//     );

//     // Camera pose relative to robot
//     April_Make2DTransform(
//         T_robot_camera_2D,
//         camera_x_robot,
//         camera_y_robot,
//         camera_theta_robot
//     );

//     // T_world_robot =
//     //     T_world_tag * inverse(T_camera_tag) * inverse(T_robot_camera)

//     Matrix3x3_Invert(T_camera_tag_2D, T_tag_camera_2D);
//     Matrix3x3_Invert(T_robot_camera_2D, T_camera_robot_2D);

//     Matrix3x3_Mult(T_world_tag_2D, T_tag_camera_2D, T_world_camera_2D);
//     Matrix3x3_Mult(T_world_camera_2D, T_camera_robot_2D, T_world_robot_2D);

//     april_robot_x = T_world_robot_2D[0][2];
//     april_robot_y = T_world_robot_2D[1][2];

//     april_robot_theta = atan2f(
//         T_world_robot_2D[1][0],
//         T_world_robot_2D[0][0]
//     );

//     NewAprilRobotPose = 1;
// }


void April_ComputeRobotPose2D_v2(
    float tagid,
    float tagx,
    float tagy,
    float tagz,
    float tagthetax,
    float tagthetay,
    float tagthetaz
) {
        
    float current_tag_world_x = 0.0f;
    float current_tag_world_y = 0.0f;
    float current_tag_world_theta = 0.0f;
    uint16_t known_tag = 1;

    switch ((int)tagid) {

    case 0:
        // Example: tag 0 is 2 ft in front of world origin
        current_tag_world_x = 2.0f;
        current_tag_world_y = 0.0f;
        current_tag_world_theta = 0.0f;
        break;

    default:
        known_tag = 0;
        break;
    }
    float r = sqrt(tagx*tagx +tagz*tagz);
    float t_tot = current_tag_world_theta + PI - tagthetaz*PI/180.0;
    float d_xa = tagz;
    float d_ya = tagx;

    april_robot_x = -d_xa * cosf(t_tot) + d_ya * sinf(t_tot) + current_tag_world_x;
    april_robot_y = -d_xa * sinf(t_tot) - d_ya * cosf(t_tot) + current_tag_world_y;
    april_robot_theta = atan2(current_tag_world_y - april_robot_y, current_tag_world_x - april_robot_x);

}
