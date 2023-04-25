#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cstring>
#include <math.h>
#include <queue>
#include <time.h>

#include "a_star.h"
#include "workspace.h"
#include "main.h"
#include "robot.h"

#define in_range(i,j) (i >= 0 && i < width && j >= 0 && j < width)

using namespace std;

/* a_star中邻居的个方位位置以及距离, 上右下左 左上 右上 右下 左下 */
const int index_dx[8] = {0, 1,  0, -1, -1, 1,  1, -1};
const int index_dy[8] = {1, 0, -1,  0,  1, 1, -1, -1};

double avg_distance;

/**
 * @brief 增加某点的热力值，当前点增加f=1，附近点增加neig_f=0.25
 * 
 * @param index_x 
 * @param index_y 
 */
void enemy_hit_F_add(int index_x, int index_y, double path_len)
{
    enemy_hit_F[index_x][index_y] += 77;
    enemy_hit_F[index_x][index_y] /= (path_len + 1.);
    for(int i = 0; i < 8; i++){
        int dx = index_x+index_dx[i];
        int dy = index_y+index_dy[i];
        if(in_range(dx, dy))
            enemy_hit_F[dx][dy] += 33;
            enemy_hit_F[index_x][index_y] /= (path_len + 1.);
    }
}


/**
 * @brief 初始化图上所有点到所有workspace的路径，障碍物除外
 *          处理了远离障碍物移动点。
 *
 */
void init_allpaths()
{
    // 初始化
    memset(allpaths_dis, 0.0, sizeof allpaths_dis);
    memset(allpaths_valid, true, sizeof allpaths_valid); // 初始化为

    // 初始化距离障碍物
    init_D();

    // 通过4个机器人判断可达与不可达工作台目标
    // 每个机器人a_star搜索所有的workspace，给机器人添加可达的工作台
    // 则每个机器人可达与不可达的workspace之间可达关系也被划分出来
    for(int i = 0; i < 4; i++) {
        vector<int> reachable_set, unreachable_set;
        Robot* rob = Robot::robots[i];
        int index_x, index_y;
        get_index_by_coordinate(rob->x, rob->y, index_x, index_y);
        Point rob_point(index_x, index_y); 

        // 遍历每一个workspace,设置该机器人起始位置到这些工作台是否可达
        // 设置机器人是否对这些 工作台是否可达
        // 设置机器人的alllpaths路径
        for(int j = 0; j < nums_of_workspaces; j++){ 
            bool valid = a_star(rob_point, j, 0, allpaths[index_x][index_y][j][0], allpaths_dis[index_x][index_y][j][0]);
            allpaths_valid[index_x][index_y][j][0] = valid;
            rob->workspace_is_reachable[j] = valid; // 该机器人到此是否可达

            // 如果可达，把带货路径也求了，不带货都走不到，带货肯定不可通
            // 处理路径点集
            if(valid){
                rob->reachable_wk_id_v.push_back(j);
                reachable_set.push_back(j);
                allpaths_valid[index_x][index_y][j][1] =
                    a_star(rob_point, j, 1, allpaths[index_x][index_y][j][1], allpaths_dis[index_x][index_y][j][1]);
                rob->workspace_is_reachable_load[j] = allpaths_valid[index_x][index_y][j][1]; // 该机器人到此是否可达
            }
            else {
                unreachable_set.push_back(j);
                rob->workspace_is_reachable_load[j] = false; // 该机器人到此是否可达
            }
        }

        // workspace之间归类可达不可达
        for(auto w1 : reachable_set) {
            index_x = Workspace::workspaces[w1]->index_x;
            index_y = Workspace::workspaces[w1]->index_y;
            // 集合之间可达不用赋值，初始化全部可达
            // 相同集合之间不可达
            for(auto w2 : unreachable_set) {
                allpaths_valid[index_x][index_y][w2][0] = 0;
                allpaths_valid[index_x][index_y][w2][1] = 0;
                // 相反 w2到w1也不可达
                allpaths_valid[Workspace::workspaces[w2]->index_x][Workspace::workspaces[w2]->index_y][w1][0] = 0;
                allpaths_valid[Workspace::workspaces[w2]->index_x][Workspace::workspaces[w2]->index_y][w1][1] = 0;
            }
        }
    }

    int path_cnt = 0; // 路径计数
    // 构建机器人和workspace的点集
    Point sours[nums_of_workspaces];
    for (int k = 0; k < nums_of_workspaces; k++)
    {
        get_index_by_coordinate(Workspace::workspaces[k]->x, Workspace::workspaces[k]->y, sours[k].index_x, sours[k].index_y);
    }

    // 初始化工作台到工作台之间的路径和机器人起始点到工作台之间的路径
    for (int n = 0; n < nums_of_workspaces; n++)
    {
        Point sour = sours[n];
        int i = sour.index_x;
        int j = sour.index_y;
        // 注意K=n每个循环只处理到没有处理(n,k)对，处理一对一次可以得到的路径直接赋值给双向的路径
        for (int k = n; k < nums_of_workspaces; k++) 
        {
            if(!allpaths_valid[i][j][k][0]) continue; // 已知不可达就跳过

            for (int l = 0; l < 2; l++)
            {
                // 寻路
                bool valid = a_star(sour, k, l, allpaths[i][j][k][l], allpaths_dis[i][j][k][l]);
                allpaths_valid[i][j][k][l] = valid;

                // 计算总的平均路径长度
                if(valid && k!=n)
                {
                    path_cnt++;
                    avg_distance += allpaths_dis[i][j][k][l];
                }
            
                // 对调起点工作台和终点工作台，直接赋值路径
                // 且对调后的终点是indanger的
                int up = 0, down = 0, left = 0, right = 0;
                if(in_danger(i, j, l, sour, up, down, left, right))
                { // 对调后indanger 就标记路径危险的
                    allpaths_valid[sours[k].index_x][sours[k].index_y][n][l] = false;
                }
                else 
                { 
                    vector<Point> temp_vec(allpaths[i][j][k][l]);
                    reverse(temp_vec.begin(), temp_vec.end());
                    allpaths[sours[k].index_x][sours[k].index_y][n][l] = temp_vec;
                    allpaths_dis[sours[k].index_x][sours[k].index_y][n][l] = allpaths_dis[i][j][k][l];
                    allpaths_valid[sours[k].index_x][sours[k].index_y][n][l] = allpaths_valid[i][j][k][l];
                    if(k!=n)
                    {
                        path_cnt++;
                        avg_distance += allpaths_dis[i][j][k][l];
                    }
                }
            }
        }
    }

    avg_distance = avg_distance/path_cnt;

    // 标记远离密集工作台集群的workspace，判定超过avg_dis的工作台占2/3以上
    for(int i = 0; i < nums_of_workspaces; i++)
    {
        int far_cnt = 0;
        int index_x = Workspace::workspaces[i]->index_x;
        int index_y = Workspace::workspaces[i]->index_y;
        for(int j = 0; j < nums_of_workspaces; j++)
        {
            if(allpaths_dis[index_x][index_y][j][0] > avg_distance*1.3)
                far_cnt++;
        }
        if(far_cnt > nums_of_workspaces*0.7) Workspace::workspaces[i]->is_far_center = 1;
        // cerr<<Workspace::workspaces[i]->id<<" is_far_center"<<Workspace::workspaces[i]->is_far_center<<endl;
    }
}


/**
 * @brief 初始化图上所有点到所有workspace的路径，障碍物除外
 *          处理了远离障碍物移动点。
 *
 */
void init_enemy_allpaths()
{
    // 初始化
    memset(enemy_allpaths_dis, 0.0, sizeof enemy_allpaths_dis);
    memset(enemy_allpaths_valid, true, sizeof enemy_allpaths_valid);
    memset(enemy_hit_F, 0, sizeof enemy_hit_F); // 敌对路径的热度

    // 初始化距离障碍物
    init_D();

    // 通过4个机器人判断可达与不可达工作台目标
    // 每个机器人a_star搜索所有的workspace，给机器人添加可达的工作台
    // 则每个机器人可达与不可达的workspace之间可达关系也被划分出来
    for(int i = 0; i < 4; i++) {
        vector<int> reachable_set, unreachable_set;
        Robot* rob = Robot::enemy_robots[i];
        int index_x, index_y;
        get_index_by_coordinate(rob->x, rob->y, index_x, index_y);
        Point rob_point(index_x, index_y); 

        // 遍历每一个workspace,设置该机器人起始位置到这些工作台是否可达
        // 设置机器人是否对这些 工作台是否可达
        // 设置机器人的alllpaths路径
        for(int j = 0; j < nums_of_enemy_workspaces; j++){ 
            if(Workspace::enemy_workspaces[j] == NULL) continue;
            Point DES(Workspace::enemy_workspaces[j]->index_x, Workspace::enemy_workspaces[j]->index_y);
            bool valid = a_star(rob_point, DES, 0, enemy_allpaths[index_x][index_y][j][0],
                                 enemy_allpaths_dis[index_x][index_y][j][0], false);
            enemy_allpaths_valid[index_x][index_y][j][0] = valid;
            rob->workspace_is_reachable[j] = valid; // 该机器人到此是否可达
            // 如果可达，把带货路径也求了，不带货都走不到，带货肯定不可通
            // 处理路径点集
            if(valid){
                reachable_set.push_back(j);
                enemy_allpaths_valid[index_x][index_y][j][1] = a_star(rob_point, DES, 1,
                    enemy_allpaths[index_x][index_y][j][1], enemy_allpaths_dis[index_x][index_y][j][1], false);
                rob->workspace_is_reachable_load[j] = enemy_allpaths_valid[index_x][index_y][j][1]; // 该机器人到此是否可达
            }
            else {
                unreachable_set.push_back(j);
                rob->workspace_is_reachable_load[j] = false; // 该机器人到此是否可达
            }
        }

        // workspace之间归类可达不可达
        for(auto w1 : reachable_set) {
            index_x = Workspace::enemy_workspaces[w1]->index_x;
            index_y = Workspace::enemy_workspaces[w1]->index_y;
            // 集合之间可达不用赋值，初始化全部可达
            // 相同集合之间不可达
            for(auto w2 : unreachable_set) {
                enemy_allpaths_valid[index_x][index_y][w2][0] = 0;
                enemy_allpaths_valid[index_x][index_y][w2][1] = 0;
                // 相反 w2到w1也不可达
                enemy_allpaths_valid[Workspace::enemy_workspaces[w2]->index_x][Workspace::enemy_workspaces[w2]->index_y][w1][0] = 0;
                enemy_allpaths_valid[Workspace::enemy_workspaces[w2]->index_x][Workspace::enemy_workspaces[w2]->index_y][w1][1] = 0;
            }
        }
    }

    // 构建机器人和workspace的点集
    Point sours[nums_of_enemy_workspaces];
    for (int k = 0; k < nums_of_enemy_workspaces; k++)
    {
        sours[k].index_x = Workspace::enemy_workspaces[k]->index_x;
        sours[k].index_y = Workspace::enemy_workspaces[k]->index_y;
    }

    // 初始化工作台到工作台之间的路径和机器人起始点到工作台之间的路径
    for (int n = 0; n < nums_of_enemy_workspaces; n++)
    {
        Point sour = sours[n];
        int i = sour.index_x;
        int j = sour.index_y;
        // 注意K=n每个循环只处理到没有处理(n,k)对，处理一对一次可以得到的路径直接赋值给双向的路径
        for (int k = n; k < nums_of_enemy_workspaces; k++) 
        {
            if(!enemy_allpaths_valid[i][j][k][0]) continue; // 已知不可达就跳过

            Point DES = sours[k];
            for (int l = 0; l < 2; l++)
            {
                // 寻路
                bool valid = a_star(sour, DES, l, enemy_allpaths[i][j][k][l], enemy_allpaths_dis[i][j][k][l], false);
                enemy_allpaths_valid[i][j][k][l] = valid;
            
                // 对调起点工作台和终点工作台，直接赋值路径
                // 且对调后的终点是indanger的
                int up = 0, down = 0, left = 0, right = 0;
                if(in_danger(i, j, l, sour, up, down, left, right))
                { // 对调后indanger 就标记路径危险的
                    enemy_allpaths_valid[sours[k].index_x][sours[k].index_y][n][l] = false;
                }
                else 
                { 
                    vector<Point> temp_vec(enemy_allpaths[i][j][k][l]);
                    reverse(temp_vec.begin(), temp_vec.end());
                    enemy_allpaths[sours[k].index_x][sours[k].index_y][n][l] = temp_vec;
                    enemy_allpaths_dis[sours[k].index_x][sours[k].index_y][n][l] = enemy_allpaths_dis[i][j][k][l];
                    enemy_allpaths_valid[sours[k].index_x][sours[k].index_y][n][l] = enemy_allpaths_valid[i][j][k][l];
                }
            }
        }
    }

    // 记录敌方路径热度
    for(int i = 0; i < nums_of_enemy_workspaces; i++)
    {
        for(int j = 0; j < nums_of_enemy_workspaces; j++)
        {
            for (int l = 0; l < 2; l++)
            {   
                if(l==0) continue;
                if(enemy_allpaths_valid[sours[i].index_x][sours[i].index_y][j][l])
                {   
                    if(workspace_sell_ok[Workspace::enemy_workspaces[i]->type][Workspace::enemy_workspaces[j]->type] ||
                        Workspace::enemy_workspaces[i]->type != 8 && Workspace::enemy_workspaces[j]->type == 9){
                            for(auto item : enemy_allpaths[sours[i].index_x][sours[i].index_y][j][l])
                                enemy_hit_F_add(item.index_x, item.index_y, 
                                    enemy_allpaths_dis[sours[i].index_x][sours[i].index_y][j][l]);
                        }
                    
                }
            }
        }
    }

    // 记录我方工作台到敌方的路径

}

/**
 * @brief 初始化本方机器人从本方所有工作台到敌方工作台的路径，不带货
 * 
 */
void init_to_enemy_allpaths()
{
    // 通过4个机器人判断可达与不可达工作台目标
    // 每个机器人a_star搜索所有的workspace，给机器人添加可达的工作台
    // 则每个机器人可达与不可达的workspace之间可达关系也被划分出来
    for(int i = 0; i < 4; i++) {
        vector<int> reachable_set, unreachable_set;
        Robot* rob = Robot::robots[i];
        int index_x, index_y;
        get_index_by_coordinate(rob->x, rob->y, index_x, index_y);
        Point rob_point(index_x, index_y); 

        // 遍历每一个workspace,设置该机器人起始位置到这些工作台是否可达
        // 设置机器人是否对这些 工作台是否可达
        // 设置机器人的alllpaths路径
        for(int j = 0; j < nums_of_enemy_workspaces; j++){ 
            Point DES(Workspace::enemy_workspaces[j]->index_x, Workspace::enemy_workspaces[j]->index_y);
            bool valid = a_star(rob_point, DES, 0,
                enemy_allpaths[index_x][index_y][j][0], enemy_allpaths_dis[index_x][index_y][j][0], false);
            enemy_allpaths_valid[index_x][index_y][j][0] = valid;
            rob->workspace_is_reachable[-j-1] = valid; // 该机器人到此是否可达
            // 处理路径点集
            if(valid){
                reachable_set.push_back(j);
                enemy_allpaths_valid[index_x][index_y][j][1] = a_star(rob_point, DES, 1,
                    enemy_allpaths[index_x][index_y][j][1], enemy_allpaths_dis[index_x][index_y][j][1], false);
                rob->workspace_is_reachable_load[-j-1] = enemy_allpaths_valid[index_x][index_y][j][1]; // 该机器人到此是否可达
            }
            else {
                unreachable_set.push_back(j);
                rob->workspace_is_reachable_load[-j-1] = false; // 该机器人到此是否可达
            }
        }

        // workspace之间归类可达不可达
        // 对敌方可到达，但对己方工作台不可到达的点，我放工作台点到敌方不可达
        for(auto wk_id : reachable_set) {
            // 集合之间可达不用赋值，初始化全部可达
            // 相同集合之间不可达
            for(int j = 0 ; j < nums_of_workspaces; j++)
            {
                index_x = Workspace::workspaces[j]->index_x;
                index_y = Workspace::workspaces[j]->index_y;
                if(!rob->workspace_is_reachable[j]){
                    enemy_allpaths_valid[index_x][index_y][wk_id][0] = 0;
                    enemy_allpaths_valid[index_x][index_y][wk_id][1] = 0;
                }
            }
        }
        // 对敌方不可到达，对己方工作台可到达的点
        for(auto wk_id : unreachable_set) {
            // 集合之间可达不用赋值，初始化全部可达
            // 相同集合之间不可达
            for(int j = 0 ; j < nums_of_workspaces; j++)
            {
                index_x = Workspace::workspaces[j]->index_x;
                index_y = Workspace::workspaces[j]->index_y;
                if(rob->workspace_is_reachable[j]){
                    enemy_allpaths_valid[index_x][index_y][wk_id][0] = 0;
                    enemy_allpaths_valid[index_x][index_y][wk_id][1] = 0;
                }
            }
        }
    }

    // 构建workspace的点集
    Point sours[nums_of_workspaces];
    for (int k = 0; k < nums_of_workspaces; k++)
    {
        sours[k].index_x = Workspace::workspaces[k]->index_x;
        sours[k].index_y = Workspace::workspaces[k]->index_y;
    }

    // 初始化工作台到工作台之间的路径和机器人起始点到工作台之间的路径
    for (int n = 0; n < nums_of_workspaces; n++)
    {
        Point sour = sours[n];
        int i = sour.index_x;
        int j = sour.index_y;
        // 注意K=n每个循环只处理到没有处理(n,k)对，处理一对一次可以得到的路径直接赋值给双向的路径
        for (int k = 0; k < nums_of_enemy_workspaces; k++) 
        {
            Point DES = Point(Workspace::enemy_workspaces[k]->index_x, Workspace::enemy_workspaces[k]->index_y);
            // for (int l = 0; l < 2; l++)
            for (int l = 0; l < 2; l++)
            {
                if(!enemy_allpaths_valid[i][j][k][l]) break; // 已知不可达就跳过
                // 寻路
                bool valid = a_star(sour, DES, l, enemy_allpaths[i][j][k][l], enemy_allpaths_dis[i][j][k][l], false);
                enemy_allpaths_valid[i][j][k][l] = valid;
            }
        }
    }
}


/**
 * @brief 通过路径长度判断得分难易，以最紧凑的一棵树来判断估计其距离
 *          若得分超过
 * @return -1 本方得分更难，0 双方得分差距不大， 1 本方得分更易
 */
int easy_score()
{
    double score_us = 0.0, score_enemy = 0.0; // total value/distance
    Workspace* wk[9] = {0}; // 保存一棵树中8个工作台 123 456 7 8/9

    /* 为本方建立一颗最紧凑的树来获得总距离 */
    Workspace* temp_wk[9] = {0};
    for(auto wk7:Workspace::workspaces_type_mp[7])
    {   // 对每个7找最近的456组合成数
        double temp_score = 0.0;
        double temp_dis4 = 10000.0;
        for(auto wk4:Workspace::workspaces_type_mp[4])
        {   // 找这个7最近的4
            if(allpaths_valid[wk4->index_x][wk4->index_y][wk7->id][1])
            {   // 路径有效
                if(allpaths_dis[wk4->index_x][wk4->index_y][wk7->id][1] < temp_dis4)
                {
                    temp_dis4 = allpaths_dis[wk4->index_x][wk4->index_y][wk7->id][1];
                    temp_wk[4] = wk4;
                }
            }
            else continue;
        }
        if(temp_wk[4] != NULL) temp_score += 7100/temp_dis4;
        // 找最近的5
        double temp_dis5 = 10000.0;
        for(auto wk5:Workspace::workspaces_type_mp[5])
        {   // 找这个7最近的5
            if(allpaths_valid[wk5->index_x][wk5->index_y][wk7->id][1])
            {   // 路径有效
                if(allpaths_dis[wk5->index_x][wk5->index_y][wk7->id][1] < temp_dis5)
                {
                    temp_dis5 =allpaths_dis[wk5->index_x][wk5->index_y][wk7->id][1];
                    temp_wk[5] = wk5;
                }
            }
            else continue;
        }
        if(temp_wk[5] != NULL) temp_score += 7800/temp_dis5;
        // 找最近的6
        double temp_dis6 = 10000.0;
        for(auto wk6:Workspace::workspaces_type_mp[6])
        {   // 找这个7最近的6
            if(allpaths_valid[wk6->index_x][wk6->index_y][wk7->id][1])
            {   // 路径有效
                if(allpaths_dis[wk6->index_x][wk6->index_y][wk7->id][1] < temp_dis6)
                {
                    temp_dis6 =allpaths_dis[wk6->index_x][wk6->index_y][wk7->id][1];
                    temp_wk[6] = wk6;
                }
            }
            else continue;
        }
        if(temp_wk[6] != NULL) temp_score += 8300/temp_dis6;

        // 7到8 9
        double temp_dis8 = 10000.0;
        for(auto wk8:Workspace::workspaces_type_mp[8])
        {   // 找这个7最近的8
            if(allpaths_valid[wk7->index_x][wk7->index_y][wk8->id][1])
            {   // 路径有效
                if(allpaths_dis[wk7->index_x][wk7->index_y][wk8->id][1] < temp_dis8)
                {
                    temp_dis8 = allpaths_dis[wk7->index_x][wk7->index_y][wk8->id][1];
                    temp_wk[8] = wk8;
                }
            }
            else continue;
        }
        for(auto wk9:Workspace::workspaces_type_mp[9])
        {   // 找这个7最近的9
            if(allpaths_valid[wk7->index_x][wk7->index_y][wk9->id][1])
            {   // 路径有效
                if(allpaths_dis[wk7->index_x][wk7->index_y][wk9->id][1] < temp_dis8)
                {
                    temp_dis8 = allpaths_dis[wk7->index_x][wk7->index_y][wk9->id][1];
                    temp_wk[8] = wk9;
                }
            }
            else continue;
        }
        if(temp_wk[8] != NULL) temp_score += 29000/temp_dis8;
        
        // 比较不同7构建的树距离
        if(temp_score > score_us)
        {
            score_us = temp_score;
            for(int i = 4; i < 9; i++) wk[i] = temp_wk[i];
        }
    }
    // 加上距离 456 最近的123
    double temp_score1 = 0.0;
    for(auto wk1:Workspace::workspaces_type_mp[1])
    {   // 到45最近的1
        double temp_s = 0.0;
        if(wk[4] && allpaths_valid[wk1->index_x][wk1->index_y][wk[4]->id][1])
        {   // 路径有效

            temp_s = 3000/allpaths_dis[wk1->index_x][wk1->index_y][wk[4]->id][1];
        }
        if(wk[5] && allpaths_valid[wk1->index_x][wk1->index_y][wk[5]->id][1])
        {   // 路径有效

            temp_s += 3000/allpaths_dis[wk1->index_x][wk1->index_y][wk[5]->id][1];
        }
        if(temp_score1 < temp_s)
        {
            temp_score1 = temp_s;
            wk[1] = wk1;
        }
    }
    double temp_score2 = 0.0;
    for(auto wk2:Workspace::workspaces_type_mp[2])
    {   // 到46最近的1
        double temp_s = 0.0;
        if(wk[4] && allpaths_valid[wk2->index_x][wk2->index_y][wk[4]->id][1])
        {   // 路径有效
            temp_s = 3200/allpaths_dis[wk2->index_x][wk2->index_y][wk[4]->id][1];
        }
        if(wk[6] && allpaths_valid[wk2->index_x][wk2->index_y][wk[6]->id][1])
        {   // 路径有效
            temp_s += 3200/allpaths_dis[wk2->index_x][wk2->index_y][wk[6]->id][1];
        }
        if(temp_score2 < temp_s)
        {
            temp_score2 = temp_s;
            wk[2] = wk2;
        }
    }
    double temp_score3 = 0.0;
    for(auto wk3:Workspace::workspaces_type_mp[3])
    {   // 到46最近的1
        double temp_s = 0.0;
        if(wk[5] && allpaths_valid[wk3->index_x][wk3->index_y][wk[5]->id][1])
        {   // 路径有效
            temp_s = 3200/allpaths_dis[wk3->index_x][wk3->index_y][wk[5]->id][1];
        }
        if(wk[6] && allpaths_valid[wk3->index_x][wk3->index_y][wk[6]->id][1])
        {   // 路径有效
            temp_s += 3200/allpaths_dis[wk3->index_x][wk3->index_y][wk[6]->id][1];
        }
        if(temp_score3 < temp_s)
        {
            temp_score3 = temp_s;
            wk[3] = wk3;
        }
    }
    score_us += (temp_score1+temp_score2+temp_score3);


    /* 为敌方建立一颗最紧凑的树来获得总距离 */
    for(int i = 0;i < 9; i++) wk[i] == 0;
    for(int i = 0;i < 9; i++) temp_wk[i] == 0;
    for(auto wk7:Workspace::enemy_workspaces_type_mp[7])
    {   // 对每个7找最近的456组合成数
        double temp_score = 0.0;
        double temp_dis4 = 10000.0;
        for(auto wk4:Workspace::enemy_workspaces_type_mp[4])
        {   // 找这个7最近的4
            if(enemy_allpaths_valid[wk4->index_x][wk4->index_y][wk7->id][1])
            {   // 路径有效
                if(enemy_allpaths_dis[wk4->index_x][wk4->index_y][wk7->id][1] < temp_dis4)
                {
                    temp_dis4 = enemy_allpaths_dis[wk4->index_x][wk4->index_y][wk7->id][1];
                    temp_wk[4] = wk4;
                }
            }
            else continue;
        }
        if(temp_wk[4] != NULL) temp_score += 7100/temp_dis4;
        // 找最近的5
        double temp_dis5 = 10000.0;
        for(auto wk5:Workspace::enemy_workspaces_type_mp[5])
        {   // 找这个7最近的5
            if(enemy_allpaths_valid[wk5->index_x][wk5->index_y][wk7->id][1])
            {   // 路径有效
                if(enemy_allpaths_dis[wk5->index_x][wk5->index_y][wk7->id][1] < temp_dis5)
                {
                    temp_dis5 =enemy_allpaths_dis[wk5->index_x][wk5->index_y][wk7->id][1];
                    temp_wk[5] = wk5;
                }
            }
            else continue;
        }
        if(temp_wk[5] != NULL) temp_score += 7800/temp_dis5;
        // 找最近的6
        double temp_dis6 = 10000.0;
        for(auto wk6:Workspace::enemy_workspaces_type_mp[6])
        {   // 找这个7最近的6
            if(enemy_allpaths_valid[wk6->index_x][wk6->index_y][wk7->id][1])
            {   // 路径有效
                if(enemy_allpaths_dis[wk6->index_x][wk6->index_y][wk7->id][1] < temp_dis6)
                {
                    temp_dis6 = enemy_allpaths_dis[wk6->index_x][wk6->index_y][wk7->id][1];
                    temp_wk[6] = wk6;
                }
            }
            else continue;
        }
        if(temp_wk[6] != NULL) temp_score += 8300/temp_dis6;

        // 7到8 9
        double temp_dis8 = 10000.0;
        for(auto wk8:Workspace::enemy_workspaces_type_mp[8])
        {   // 找这个7最近的8
            if(enemy_allpaths_valid[wk7->index_x][wk7->index_y][wk8->id][1])
            {   // 路径有效
                if(enemy_allpaths_dis[wk7->index_x][wk7->index_y][wk8->id][1] < temp_dis8)
                {
                    temp_dis8 = enemy_allpaths_dis[wk7->index_x][wk7->index_y][wk8->id][1];
                    temp_wk[8] = wk8;
                }
            }
            else continue;
        }
        for(auto wk9:Workspace::enemy_workspaces_type_mp[9])
        {   // 找这个7最近的9
            if(enemy_allpaths_valid[wk7->index_x][wk7->index_y][wk9->id][1])
            {   // 路径有效
                if(enemy_allpaths_dis[wk7->index_x][wk7->index_y][wk9->id][1] < temp_dis8)
                {
                    temp_dis8 = enemy_allpaths_dis[wk7->index_x][wk7->index_y][wk9->id][1];
                    temp_wk[8] = wk9;
                }
            }
            else continue;
        }
        if(temp_wk[8] != NULL) temp_score += 29000/temp_dis8;
        
        // 比较不同7构建的树距离
        if(temp_score > score_enemy)
        {
            score_enemy = temp_score;
            for(int i = 4; i < 9; i++) wk[i] = temp_wk[i];
        }
    }
    // 加上距离 456 最近的123
    temp_score1 = 0.0;
    for(auto wk1:Workspace::enemy_workspaces_type_mp[1])
    {   // 到45最近的1
        double temp_s = 0.0;
        if(wk[4] && enemy_allpaths_valid[wk1->index_x][wk1->index_y][wk[4]->id][1])
        {   // 路径有效

            temp_s = 3000/enemy_allpaths_dis[wk1->index_x][wk1->index_y][wk[4]->id][1];
        }
        if(wk[5] && enemy_allpaths_valid[wk1->index_x][wk1->index_y][wk[5]->id][1])
        {   // 路径有效

            temp_s += 3000/enemy_allpaths_dis[wk1->index_x][wk1->index_y][wk[5]->id][1];
        }
        if(temp_score1 < temp_s)
        {
            temp_score1 = temp_s;
            wk[1] = wk1;
        }
    }
    temp_score2 = 0.0;
    for(auto wk2:Workspace::enemy_workspaces_type_mp[2])
    {   // 到46最近的1
        double temp_s = 0.0;
        if(wk[4] && enemy_allpaths_valid[wk2->index_x][wk2->index_y][wk[4]->id][1])
        {   // 路径有效
            temp_s = 3200/enemy_allpaths_dis[wk2->index_x][wk2->index_y][wk[4]->id][1];
        }
        if(wk[6] && enemy_allpaths_valid[wk2->index_x][wk2->index_y][wk[6]->id][1])
        {   // 路径有效
            temp_s += 3200/enemy_allpaths_dis[wk2->index_x][wk2->index_y][wk[6]->id][1];
        }
        if(temp_score2 < temp_s)
        {
            temp_score2 = temp_s;
            wk[2] = wk2;
        }
    }
    temp_score3 = 0.0;
    for(auto wk3:Workspace::enemy_workspaces_type_mp[3])
    {   // 到46最近的1
        double temp_s = 0.0;
        if(wk[5] && enemy_allpaths_valid[wk3->index_x][wk3->index_y][wk[5]->id][1])
        {   // 路径有效
            temp_s = 3200/enemy_allpaths_dis[wk3->index_x][wk3->index_y][wk[5]->id][1];
        }
        if(wk[6] && enemy_allpaths_valid[wk3->index_x][wk3->index_y][wk[6]->id][1])
        {   // 路径有效
            temp_s += 3200/enemy_allpaths_dis[wk3->index_x][wk3->index_y][wk[6]->id][1];
        }
        if(temp_score3 < temp_s)
        {
            temp_score3 = temp_s;
            wk[3] = wk3;
        }
    }
    score_enemy += (temp_score1+temp_score2+temp_score3);

    cerr<<"score_us"<<score_us<<endl;
    cerr<<"score_enemy"<<score_enemy<<endl;


    if(1.2 * score_us < score_enemy ) return -1;
    else if(1.2 * score_enemy < score_us) return 1;
    return 0;
}


/*****************************TEST*************************************/

// int main() {
//     // mook graph
//     nums_of_workspaces = 0;
//     for(int i = 0; i < width; i++)
//         for(int j = 0; j < width; j++){
//             cin>>graph[i][j];
//             // mook worksapces
//             if (graph[i][j] <= '9' && graph[i][j] >= '1')
//             { // 工作台初始化点
//                 (Workspace::workspaces).push_back(new Workspace((graph[i][j]-'0'), j * 0.5 + 0.25, 50 - i * 0.5 - 0.25, i, j, 'b'));
//                 nums_of_workspaces++;
//             }
//             else if (graph[i][j] == 'A')
//             { // 机器人初始化点
//                 (Robot::robots).push_back(new Robot(j * 0.5 + 0.25, 50 - i * 0.5 - 0.25, 'b'));
//             }
//             else if (graph[i][j] <= 'i' && graph[i][j] >= 'a')
//             {
//                 (Workspace::enemy_workspaces).push_back(new Workspace((graph[i][j]-'0'), j * 0.5 + 0.25, 50 - i * 0.5 - 0.25, i, j, 'r'));
//                 nums_of_enemy_workspaces++;
//             }
//             else if (graph[i][j] == 'B')
//             { // 机器人初始化点
//                 (Robot::enemy_robots).push_back(new Robot(j * 0.5 + 0.25, 50 - i * 0.5 - 0.25, 'r'));
//             }
//         }

//     // 计算时间测试
//     clock_t start, end;
//     start = clock();

//     int test_workspace_id = 0;

//     // init_D();

//     // int index_x,index_y; 
//     // get_index_by_coordinate(Workspace::workspaces[0]->x, Workspace::workspaces[0]->y, index_x, index_y);
//     // vector<Point> path;

//     // // 单路径寻路测试，将路径用X标注在地图，使用map4
//     // double path_dis;
//     // cout<<a_star(Point(index_x, index_y), test_workspace_id, 1, path, path_dis)<<endl;;
//     // for(auto item: path){
//     //     cout<<item.index_x<<","<<item.index_y<<";  "<<item.x<<","<<item.y<<endl;
//     //     graph[item.index_x][item.index_y] = '!';
//     // }
//     // for(int i = 0; i < width; i++) {
//     //     for(int j = 0; j < width; j++){
//     //         cout<<graph[i][j];
//     //     }
//     //     cout<<endl;
//     // }


//     // 测试robot0起始位置到某个工作台的路径搜索结果，测试allpaths结果
//     init_allpaths();

//     int index_x,index_y; 
//     get_index_by_coordinate(Robot::robots[0]->x, Robot::robots[0]->y, index_x, index_y);
//     cout<<index_x<<" "<<index_y<<endl;
//     for(auto item: allpaths[index_x][index_y][test_workspace_id][0]){
//         cout<<item.index_x<<","<<item.index_y<<";  "<<item.x<<","<<item.y<<endl;
//         graph[item.index_x][item.index_y] = '!';
//     }
//     if(allpaths[index_x][index_y][test_workspace_id][0].size() == 0) cout<<0<<endl;
//     else cout<<1<<endl;

//     for(int i = 0; i < width; i++) {
//         for(int j = 0; j < width; j++){
//             cout<<graph[i][j];
//         }
//         cout<<endl;
//     }
//     cout<<endl;


// //    // 测试敌对init
// //    init_enemy_allpaths();

// //     int test_enemy_wk_id = 3;
// //     int test_enemy_wk_id_des = 6;
// //     int index_x,index_y; 
// //     index_x = Workspace::enemy_workspaces[test_enemy_wk_id]->index_x;
// //     index_y = Workspace::enemy_workspaces[test_enemy_wk_id]->index_y;
// //     for(auto item: enemy_allpaths[index_x][index_y][test_enemy_wk_id_des][1]){
// //         cout<<item.index_x<<","<<item.index_y<<";  "<<item.x<<","<<item.y<<endl;
// //         graph[item.index_x][item.index_y] = '!';
// //     }
// //     if(enemy_allpaths_valid[index_x][index_y][test_enemy_wk_id_des][1] == 0) cout<<0<<endl;
// //     else cout<<1<<endl;
// //     for(int i = 0; i < width; i++) {
// //         for(int j = 0; j < width; j++){
// //             cout<<graph[i][j];
// //         }
// //         cout<<endl;
// //     }

//     // 测试己方工作台到敌方工作台
//     // init_to_enemy_allpaths();

//     // int test_my_wk_id = 7;
//     // int test_enemy_wk_id = 2;
//     // int index_x,index_y; 
//     // index_x = Workspace::workspaces[test_my_wk_id]->index_x;
//     // index_y = Workspace::workspaces[test_my_wk_id]->index_y;
//     // for(auto item: enemy_allpaths[index_x][index_y][test_enemy_wk_id][0]){
//     //     cout<<item.index_x<<","<<item.index_y<<";  "<<item.x<<","<<item.y<<endl;
//     //     graph[item.index_x][item.index_y] = '!';
//     // }
//     // if(enemy_allpaths_valid[index_x][index_y][test_enemy_wk_id][0] == 0) cout<<0<<endl;
//     // else cout<<1<<endl;
//     // for(int i = 0; i < width; i++) {
//     //     for(int j = 0; j < width; j++){
//     //         cout<<graph[i][j];
//     //     }
//     //     cout<<endl;
//     // }
    

//     // 结束时间
//     end = clock();
//     double elapsedTime = static_cast<double>(end-start) / CLOCKS_PER_SEC ;
//     //clock()以毫秒的形式展现，因此需要除以 CLOCKS_PER_SEC 来实现转换
//     //static_cast<double>的作用是将结果转换为double类型
//     printf("CPU PROCESSING TIME: %f",elapsedTime);
//     return 0;
// }