#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    float dtheta_gyro = gyro_z * dt;
    float dtheta_odom = wz * dt;

    float dG_O = dtheta_gyro - dtheta_odom;

    float dtheta_threshold = 0.125 * (PI/180) * 0.1;

    float abs_GO = fabs(dG_O);

    float theta_curr = 0; // Initialize theta_curr

    if (abs_GO > dtheta_threshold){
        theta_curr = *theta + dtheta_gyro * dt;
    } else if (abs_GO < dtheta_threshold){
        theta_curr = *theta + dtheta_odom * dt;
    }

    // Update pose
    *x += vx * dt * cos(theta_curr) - vy * dt * sin(theta_curr);
    *y += vx * dt * sin(theta_curr) + vy * dt * cos(theta_curr);
    // *theta += wz * dt;
    *theta = theta_curr;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;

    return;
}