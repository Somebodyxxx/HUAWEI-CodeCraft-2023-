#ifndef RADAR_H
#define RADAR_H

#include "a_star.h"
#include "robot.h"
#include "main.h"
#include <math.h>

#define PI 3.14159265358979323846

#define RADAR_IS_US 1.7
#define RADAR_EXIST_ENERMY 1.7
#define RADAR_LINE_LEN 30.
#define RADAR_DIS_BIAS 0.1


bool radar_is_exist_enermy(const Point& radar_p);

bool radar_is_wall(const Point& radar_p);

bool radar_is_us(const Point& radar_p, Robot* rot);

Point radar_get_point(Robot* rot, int angle, double dis_bias = 0.);

double get_dis(const Point& a, const Point& b);

// 在main中调用
void radar_update_enermy_robot();

#endif