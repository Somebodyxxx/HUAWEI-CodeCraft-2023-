#include "radar.h"
#include<iostream>
using namespace std;
double get_dis(const Point& a, const Point& b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool radar_is_us(const Point& radar_p, Robot* this_rot) {
	for (Robot* rot : Robot::robots) {
		if(rot == this_rot) continue;
		if (get_dis(Point(rot->x, rot->y, 1), radar_p) < 0.9)
			return true;
	}
	return false;
}

bool radar_is_exist_enermy(const Point& radar_p) {
	if (Robot::enemy_robots.size() >= 4) {
		return true;
	}
	for (Robot* rot : Robot::enemy_robots) {
		if (get_dis(Point(rot->x, rot->y, 1), radar_p) < 0.9)
			return true;
	}
	return false;
}

Point radar_get_point(Robot* rot, int angle, double dis_bias) {
	double radar_line_len = rot->radar[angle] + dis_bias;
	double radian = (rot->rotation * 180. / PI + double(angle)) * PI / 180.;
	double delta_x = radar_line_len * cos(radian);
	double delta_y = radar_line_len * sin(radian);
	return Point(rot->x + delta_x, rot->y + delta_y, 1);
}

bool radar_is_wall(const Point& radar_p) {
	// int level = 1;
	// for(int i=-level; i<=level; i++){
	// 	for(int j=-level; j<=level; j++){
	// 		if(is_alarm(radar_p.index_x + i, radar_p.index_y + j)){
	// 			return true;
	// 		}
	// 	}
	// }
	// return false;

	int ul, ur, dl, dr;
	if(is_alarm(radar_p.index_x, radar_p.index_y) || 
		in_danger(radar_p.index_x, radar_p.index_y, 1, Point(-1, -1),ul, ur, dl, dr)){
			return true;
		}
	return false;
}

void radar_update_enermy_robot() {
	// 每帧清零
	for (int i = 0; i < Robot::enemy_robots.size(); i++) {
		delete Robot::enemy_robots[i];
	}
	Robot::enemy_robots.clear();

	for (Robot* rot : Robot::robots) {
		if (Robot::enemy_robots.size() >= 4) break;
		for (int angle = 0; angle < 360; angle++) {
			if (Robot::enemy_robots.size() >= 4) break;

			// 长度大于30的探测线忽略
			if (rot->radar[angle] > 30.) continue;

			// +0.1 判断障碍物
			Point radar_cur = radar_get_point(rot, angle, 0.1);
			if (radar_is_wall(radar_cur)) continue;
			if (radar_is_us(radar_cur, rot)) continue;
			if (radar_is_exist_enermy(radar_cur)) continue;

			// 记录离自己最近的那条雷达射线, +0.2求点作为敌方坐标
			double min_radar_len = rot->radar[angle];
			int record_angle = angle;

			while (true) {
				if (angle < 359) angle++;
				else break;

				radar_cur = radar_get_point(rot, angle, 0.1);
				if (radar_is_wall(radar_cur)) break;
				if (radar_is_us(radar_cur, rot)) break;
				if (radar_is_exist_enermy(radar_cur)) break;

				if (rot->radar[angle] < min_radar_len) {
					record_angle = angle;
					min_radar_len = rot->radar[angle];
				}
			}

			Point ep = radar_get_point(rot, record_angle, 0.1);
			char ec = rot->color == 'b' ? 'r' : 'b';
			Robot* enermy = new Robot(ep.x, ep.y, ec);
			Robot::enemy_robots.push_back(enermy);
		}
	}
}
