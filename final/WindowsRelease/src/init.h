#ifndef INIT_H
#define INIT_H

#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

extern double avg_distance;

/**
 * @brief 初始化图上所有点到所有workspace的路径，障碍物除外
 *          处理了远离障碍物移动点。
 *
 */
void init_allpaths();


/**
 * @brief 初始化图上所有敌对工作台点到所有workspace的路径，障碍物除外
 *          处理了远离障碍物移动点。
 *
 */
void init_enemy_allpaths();

/**
 * @brief 初始化本方机器人从本方所有工作台到敌方工作台的路径，不带货
 * 
 */
void init_to_enemy_allpaths();

/**
 * @brief 通过路径长度判断得分难易，以最紧凑的一棵树来判断估计其距离
 *          若得分超过
 * @return -1 本方得分更难，0 双方得分差距不大， 1 本方得分更易
 */
int easy_score();



#endif