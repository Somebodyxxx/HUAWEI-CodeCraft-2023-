#include "robot.h"
#include "workspace.h"
#include "main.h"
#include "a_star.h"
#include "init.h"

#include <math.h>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <map>
#include <stdio.h>
#include <cstring>

using namespace std;
#define PI 3.14159265358979323846
#define in_range(i,j) (i >= 0 && i < width && j >= 0 && j < width)

int Robot::money = 200000;
int Robot::nums_of_attack_robot = 0;
int far_continue_cnt = 0; // 记录跳过远处132买点的次数，跳过帧数多了说明拥挤，去攻击

vector<Robot *> Robot::robots;
vector<Robot *> Robot::enemy_robots;

/**
 * @brief 判断是否有路径点冲突：点相同地方 且 距离源点有冲突
 * 
 * @param mp 
 * @param current_path 
 * @param dis_vec 
 * @return true 
 * @return false 
 */

bool point_near(Point p1,Point p2){// 2 容易堵着自己人，碰屁股   ;best = 3 图2：2
    if(abs(p1.index_x - p2.index_x) + abs(p1.index_y - p2.index_y) <= point_near_value)//曼哈顿距离<=2视为邻格
    {
        return true;
    }
    return false;
}

bool Robot::assign_robot_to_attack(Workspace* wk){
    int attack_wk_id = -wk->id - 1;
    vector<Point> attack_path;
    double attack_dist;
    if(workspace_is_reachable[attack_wk_id]){//机器人不带货可达工作点
        if(wk->in_corner() >= 2){
            // cerr<<"test 1..."<<endl;
            //边角工作台，不买，直接去
            if(enemy_allpaths[index_x][index_y][wk->id][0].size()){
                attack_path = enemy_allpaths[index_x][index_y][wk->id][0];
                //在move 判断incorner的类型，偏移最终要占据的点
                task.push(make_pair(attack_wk_id,attack_path));
                return true;
            }else{
                // cerr<<"assign rot to attack error1..."<<endl;
            }
        }else if(color == 'b'){//非边角工作台，看下带货能不能到
            int target_buy = -1;
            double cost = 10000.0;
            for(Workspace* wk_buy : Workspace::workspaces){
                if(wk_buy->type != 1){//TODO:考虑也可以买类型2、类型3
                    continue;
                }
                // 不可达点跳过
                if(!workspace_is_reachable[wk_buy->id]){//不带货到买点是否可达
                    continue;
                }
                if(!enemy_allpaths_valid[wk_buy->index_x][wk_buy->index_y][wk->id][1]){//从买点带货到目标攻击点是否可达
                    if(wk_buy->id == 0 && color == 'r'){//TODO:bug 
                        // cerr<<"error  test 2..."<<endl;
                    }
                    continue;
                }
                double buy_dis = allpaths_dis[index_x][index_y][wk_buy->id][0];
                double sell_dis = enemy_allpaths_dis[wk_buy->index_x][wk_buy->index_y][wk->id][1];
                double cur_dis = buy_dis + sell_dis;
                if(cur_dis < cost){
                    target_buy = wk_buy->id;
                    cost = cur_dis;
                }
            }

            if(target_buy != -1){//可以带货攻击
                vector<Point> buy_path = allpaths[index_x][index_y][target_buy][0];
                attack_path = enemy_allpaths[Workspace::workspaces[target_buy]->index_x][Workspace::workspaces[target_buy]->index_y][wk->id][1];
                task.push(make_pair(target_buy,buy_path));
                task.push(make_pair(attack_wk_id,attack_path));
                return true;
            }else {
                //蓝方不带货 不占据进攻
            }
        }else{
            //不满足红蓝方的选点条件
        }
    }else{
        //不可达
    }
    // cerr<<"assign rot to attack error..."<<endl;
    return false;
}

void Robot::stand_attack(){

    //TODO:nums_of_robot
    //先统计敌方工作台各个类型的数量 enemy_workspaces_type_mp
    int num1 = Workspace::enemy_workspaces_type_mp[1].size();
    int num2 = Workspace::enemy_workspaces_type_mp[2].size();
    int num3 = Workspace::enemy_workspaces_type_mp[3].size();
    int num4 = Workspace::enemy_workspaces_type_mp[4].size();
    int num5 = Workspace::enemy_workspaces_type_mp[5].size();
    int num6 = Workspace::enemy_workspaces_type_mp[6].size();
    int num7 = Workspace::enemy_workspaces_type_mp[7].size();
    int num8 = Workspace::enemy_workspaces_type_mp[8].size();
    int num9 = Workspace::enemy_workspaces_type_mp[9].size();

    // 首先处理corner情况的
    int nums_corner[10]; // 每个类型的corner
    memset(nums_corner, 0, sizeof nums_corner);
    for(int i = 1; i <= 9; i++)
    {
        for(auto wk:Workspace::enemy_workspaces_type_mp[i])
        {
            if(wk->in_corner() >= 2) nums_corner[i]++;
            else // 只要有一个不邻墙角，这个数组就设置为0
            {
                nums_corner[i] = 0; 
                break;
            }
        }
    }

    if(nums_corner[3] == 1) // 3号台只有1个且都为邻墙角
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[3])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    if(nums_corner[2] == 1)
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[2])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    if(nums_corner[1] == 1)
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[1])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    if(nums_corner[6] == 1)
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[6])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    if(nums_corner[5] == 1)
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[5])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    if(nums_corner[4] == 1)
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[4])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    if(nums_corner[7] + nums_corner[9] == 1 && num4+num5+num6<13)
    {
        Workspace* target_wk = 0;
        for(auto wk:Workspace::enemy_workspaces_type_mp[7])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        for(auto wk:Workspace::enemy_workspaces_type_mp[9])
        {
            if(wk->in_corner() >= 2)
            {
                target_wk = wk;
                break;
            }
        }
        if(target_wk == 0) return;
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }
    // 开始判断两个工作台都相邻墙角
    if(nums_corner[3] == 2)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[3])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if(nums_corner[2] == 2)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[2])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if(nums_corner[1] == 2)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[1])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if(nums_corner[6] == 2)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[6])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if(nums_corner[5] == 2)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[5])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if(nums_corner[4] == 2)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[4])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if((nums_corner[7] + nums_corner[9]) && nums_corner[7] + nums_corner[9] <= 2 && num4+num5+num6<13)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[7])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        for(auto wk : Workspace::enemy_workspaces_type_mp[9])
        {
            if(wk->in_corner() < 2) continue;
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }

    // 对于普通工作台占领
    if((num7 + num9) && num7 + num9 <= 2 && num4+num5+num6<13)
    {
        int count = 0; //count项任务，分配给robots.size个机器人
        for(auto wk : Workspace::enemy_workspaces_type_mp[7])
        {
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        for(auto wk : Workspace::enemy_workspaces_type_mp[9])
        {
            for(Robot* rot : robots){
                if(rot->is_stand_attacking) continue;
                if(rot->assign_robot_to_attack(wk)){ // 且可分配任务
                    rot->is_stand_attacking = 1;
                    count++;
                    break;
                }
            }
        }
        if(count != 2){//任务没分配完
            for(Robot* rot : robots){
                while(rot->task.size()){
                    rot->task.pop();
                }
                rot->is_stand_attacking = 0;
            }
        }else{
            nums_of_attack_robot += 2;
            return;
        }
    }
    if(num7 == 1)
    {
        Workspace* target_wk = Workspace::enemy_workspaces_type_mp[7][0];
        for(Robot* rot : robots){
            if(rot->assign_robot_to_attack(target_wk)){
                rot->is_stand_attacking = 1;
                nums_of_attack_robot++;
                // break;
                return;
            }
        }
    }



    

    // /***********************先考虑只派一个机器人去占据的情况*************************/
    // //TODO:考虑边角工作台优先占据
    // //TODO:考虑集群优先占据
    // //1 - 7只有一个TODO:优先顺序好像有问题？
    // if(num9 == 1 && num7 == 0){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[9][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num3 == 1){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[3][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num2 == 1){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[2][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num1 == 1){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[1][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num6 == 1){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[6][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num5 == 1){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[5][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num4 == 1){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[4][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num7 == 1 && num4 && num5 && num6){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[7][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // if(num8 == 1 && num9 == 0 && num7 && num4 && num5 && num6){
    //     Workspace* target_wk = Workspace::enemy_workspaces_type_mp[8][0];
    //     for(Robot* rot : robots){
    //         if(rot->assign_robot_to_attack(target_wk)){
    //             rot->is_stand_attacking = 1;
    //             nums_of_attack_robot++;
    //             // break;
    //             return;
    //         }
    //     }
    // }
    // /*****************************考虑派两个机器人去占据进攻*********************************/
    // //先判断类型7和9 ， 7和8 数量是否小于2.
    // if(num3 == 2){
    //     int count = num3;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[3][index])){
    //                 rot->is_stand_attacking = 1;
    //                 index++;
    //                 count--;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[3].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num3;
    //         return;
    //     }
    // }
    // if(num2 == 2){
    //     int count = num2;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[2][index])){
    //                 rot->is_stand_attacking = 1;
    //                 index++;
    //                 count--;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[2].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num2;
    //         return;
    //     }
    // }
    // if(num1 == 2){
    //     int count = num1;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[1][index])){
    //                 rot->is_stand_attacking = 1;
    //                 index++;
    //                 count--;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[1].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num1;
    //         return;
    //     }
    // }
    // if(num6 == 2){
    //     int count = num6;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[6][index])){
    //                 index++;
    //                 count--;
    //                 rot->is_stand_attacking = 1;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[6].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num6;
    //         return;
    //     }
    // }
    // if(num5 == 2){
    //     int count = num5;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[5][index])){
    //                 index++;
    //                 count--;
    //                 rot->is_stand_attacking = 1;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[5].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num5;
    //         return;
    //     }
    // }
    // if(num4 == 2){
    //     int count = num4;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[4][index])){
    //                 index++;
    //                 count--;
    //                 rot->is_stand_attacking = 1;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[4].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num4;
    //         return;
    //     }
    // }
    // if(num7 == 2 && num9 == 0){
    //     //派机器人去占据
    //     int count = num7;
    //     int index = 0;
    //     while(count){//count项任务，分配给robots.size个机器人
    //         for(Robot* rot : robots){
    //             if(rot->assign_robot_to_attack(Workspace::enemy_workspaces_type_mp[7][index])){
    //                 index++;
    //                 count--;
    //                 rot->is_stand_attacking = 1;
    //                 if(index >= Workspace::enemy_workspaces_type_mp[7].size()){
    //                     break;
    //                 }
    //             }
    //         }
    //         break;
    //     }
    //     if(count){//任务没分配完
    //         for(Robot* rot : robots){
    //             while(rot->task.size()){
    //                 rot->task.pop();
    //             }
    //             rot->is_stand_attacking = 0;
    //         }
    //     }else{
    //         nums_of_attack_robot += num7;
    //         return;
    //     }
    // }
}

void Robot::init_frame(vector<string> vec)
{
    is_near_workspace = stoi(vec[0]);

    load_id = atoi(vec[1].c_str());
    if (load_id == 0)
    {
        r = 0.45;
    }
    else
    {
        r = 0.53;
    }

    holdtime = atoi(vec[2].c_str());
    collidetimes = atoi(vec[3].c_str());
    w = stod(vec[4]);
    v[0] = stod(vec[5]);
    v[1] = stod(vec[6]);
    rotation = stod(vec[7]);
    x = stod(vec[8]);
    y = stod(vec[9]);

    get_index_by_coordinate(x,y,index_x,index_y);
    update_all_robots_next_path();

    BFS_fail = 0;

    near_face = 0;
    for(Robot* rot : enemy_robots){
        if(sqrt(pow(x - rot->x,2) + pow(y - rot->y, 2)) < r + 0.53 +0.1){
            near_face =1;
            break;
        }
    }

    near_wall =0;
    bool outloop_break = 0;
    for(int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            if(graph[index_x+i][index_y+j] == '#'){
                near_wall = 1;
                outloop_break = 1;
                break;
            }
        }
        if(outloop_break){
            break;
        }
    }
}

void Robot::stand_space_attack(){
    if(!task.empty()){
        return;
    }
    if(mapID == 1){
        nums_of_attack_threshold = 0;
        if(color == 'r'){
            nums_of_attack_threshold = 0;
        }   
    }

    if(nums_of_attack_robot >= nums_of_attack_threshold){//1表示 当前攻击的机器人数量1，即一共可以派2个去进攻
        return;
    }
    
    if(!(frameID > 50  &&  (avoid_times / (double)frameID > attack_threshold) && load_id == 0)){
        return;
    }
    cerr<<id<<" + "<<avoid_times << " +"<< frameID<<"  double frameID="<< (double)frameID <<endl;

    vector<int> temp;
    Point attack_point;
    if(!select_eid_to_attack("ss", temp, attack_point, 0, id, 0)){
        return;
    }

    if(color == 'r'){
        if(stand_space_task(attack_point,12000,0,Point(index_x,index_y),2000)){
            nums_of_attack_robot++;
        }
        return;
    }

    //先把数据结构找回来
    vector<Workspace* > reachable_ws;
    for(int i=0; i < all_hot_stand_space.size(); i++){
        if(all_hot_stand_space[i].hss.index_x == attack_point.index_x &&
            all_hot_stand_space[i].hss.index_y == attack_point.index_y){
                reachable_ws = all_hot_stand_space[i].reachable_ws_12;
            }
    }

    // 找个地方去买1
    double attack_cost = 1e9;
    int attack_buy_target = -1;
    vector<Point> attack_buy_path;
    for(Workspace* wk : reachable_ws){
        if(!wk->is_safe){
            continue;
        }
        if(wk->type != 1 && wk->type != 2){  
            continue;
        }
        // 不可达点跳过
        if(!workspace_is_reachable[wk->id]){
            continue;
        } 

        bool continue_out_loop = 0;
        for(auto rot : enemy_robots){
            if(abs(rot->index_x - wk->index_x) + abs(rot->index_y - wk->index_y) <= 2){
                //工作台被敌方机器人占据
                continue_out_loop = 1;
                break;
            }
        }
        if(continue_out_loop){
            continue;
        }
        double dist_buy = 10000.0;
        vector<Point> temp_buy = allpaths[index_x][index_y][wk->id][0];
        if(temp_buy.empty()){
            if(a_star(Point(index_x,index_y),wk->id,0,temp_buy,dist_buy)){
                allpaths[index_x][index_y][wk->id][0] = temp_buy;
                allpaths_dis[index_x][index_y][wk->id][0] = dist_buy;
            }else{//不可达
                continue;
            }
        }
        dist_buy = allpaths_dis[index_x][index_y][wk->id][0];//dist赋值
        // 尽量不选2买
        if(wk->type == 2) dist_buy = dist_buy * 100.;

        if(dist_buy < attack_cost){
            attack_cost = dist_buy; 
            attack_buy_target = wk->id;
            attack_buy_path = temp_buy;
        }
    }

    
    // 如果能买到1, 并且能找到带货攻击热力点
    if(attack_buy_target != -1){
        task.push(make_pair(attack_buy_target,attack_buy_path));
        // 不会失败, 初始化都是带货可达的
        if(stand_space_task(attack_point,12000,1,Point(Workspace::workspaces[attack_buy_target]->index_x,Workspace::workspaces[attack_buy_target]->index_y),2000)){
            nums_of_attack_robot++;
        }
    }
    else if(stand_space_task(attack_point,12000,0,Point(index_x,index_y),2000)){
            nums_of_attack_robot++;
    }     
    else{
        // 无法进攻
        avoid_times = 0.;
    }
}

void Robot::simple_seek(){
    if(!task.empty()){
        return;
    }
    if(load_id){
        reseek();
        return;
    }
    Workspace::init_impact_factor();
    
    //接任务
    double cost_sum_dist = 100000.0;
    double current_sum_dist;
    double nine_cost = 100000.0;
    vector<Point> path_buy;
    vector<Point> path_sell;
    vector<Point> path_sell_nine;
    int target1 = -1;
    int target2 = -1;
    int target2_nine = -1;
    vector<Point> temp_buy;
    double dist_buy = 100000.0;
    double dist_sell = 100000.0;
    // double dist_sell_nine = 100000.0;
    double time_buy,time_sell,time_sell_nine;
    double record_time_buy,record_time_sell;

    for(Workspace* wk : Workspace::workspaces){//买点
        //
        if(!wk->is_safe){ // TODO: 没台买卖了还是会去买货卖
            continue;
        }
        if (wk->type == 8 || wk->type == 9) {
            continue;
        }
        // 不可达点跳过
        if(!workspace_is_reachable[wk->id]){
            continue;
        } 
        // TODO:: 预定位对123关闭
        if(wk->reserved[wk->type] == 1 && wk->type != 1 && wk->type != 2 && wk->type != 3){//被预定
            continue;
        }

        // bool continue_out_loop = 0;
        // for(auto rot : enemy_robots){//TODO:可以关？
        //     if(abs(rot->index_x - wk->index_x) + abs(rot->index_y - wk->index_y) <= 2){
        //         //工作台被敌方机器人占据
        //         continue_out_loop = 1;
        //         break;
        //     }
        // }
        // if(continue_out_loop){
        //     continue;
        // }

        temp_buy =  allpaths[index_x][index_y][wk->id][0];

        if(temp_buy.empty()){
            if(a_star(Point(index_x,index_y),wk->id,0,temp_buy,dist_buy)){ // TODO: 对于非工作台点总a_star掉帧
                allpaths[index_x][index_y][wk->id][0] = temp_buy;
                allpaths_dis[index_x][index_y][wk->id][0] = dist_buy;
            }else{
                continue;
            }
        }
        dist_buy = allpaths_dis[index_x][index_y][wk->id][0];//dist赋值

        time_buy = get_time_by_dist(dist_buy);

        if(wk->reserved[wk->type] == 1){
            time_buy = time_buy + 250.0;//权衡掉同时去买的弊端 20
        }

        if (wk->preduct_state == 1 || ( (time_buy + wait_product > wk->remain_preduct_time) && (wk->remain_preduct_time != -1) )){//没生产好
            //没问题
        }else if(wk->type != 1 && wk->type != 2 && wk->type != 3){
            continue;
        }

        for (int target2Id : wk->next_workspace){
            //TODO:有safe了这里可以不判断？加了反而影响最后没货取时不取货的问题？
            int enemy_num = 0;
            for(auto rot : enemy_robots){
                if(abs(rot->index_x - Workspace::workspaces[target2Id]->index_x) + abs(rot->index_y - Workspace::workspaces[target2Id]->index_y) <= 2){
                    //工作台被敌方机器人占据
                    enemy_num ++;
                }
            }
            //放宽过滤条件
            if((enemy_num >= 2 ) || (enemy_num && color == 'r') || 
            ((Workspace::workspaces[target2Id]->in_corner()>=2) && (enemy_num)) ){
                Workspace::workspaces[target2Id]->is_safe = 0;
            }

            // if(!Workspace::workspaces[target2Id]->is_safe && Workspace::workspaces[target2Id]->in_corner()>=2){
            //     continue;
            // }
            if(!Workspace::workspaces[target2Id]->is_safe){
                continue;
            }
            if(allpaths_valid[wk->index_x][wk->index_y][target2Id][1]){
                //可达
            }else{
                continue;
            }
            if(Workspace::workspaces[target2Id]->reserved[wk->type] == 1)//该原料格被预定
            {
                continue;
            }
            //一次只能有一个4 5 6 存在于场上。
            // if(Workspace::workspaces[target2Id]->type == 7 && 
            //     !Workspace::workspaces[target2Id]->is_resouce_no_reserved() && num_of_seven == 1){
            //         continue;
            //     }
            
            

            dist_sell = allpaths_dis[wk->index_x][wk->index_y][target2Id][1];//dist赋值，工作点之间，必定有值
            

            //优化，使得卖给4 5 6平均。
            if(num_of_seven){
                // cerr<< "hello 1.."<<endl;
                // 获得456类型台中最大和最小访问次数
                int max_visit = *max_element(Workspace::visit_times, Workspace::visit_times + 3);
                int min_visit = *min_element(Workspace::visit_times, Workspace::visit_times + 3);
                // 当前是最大访问次数工作台且比最小访问次数大于2了，且当前全空，就直接跳过，不考虑给他送了
                if (Workspace::visit_times[Workspace::workspaces[target2Id]->type - 4] == max_visit
                    && ((max_visit - min_visit) > 2)) {
                        // if(Workspace::workspaces[target2Id]->resource_state_o == 0){
                        //     continue;
                        // }
                        continue;
                }
            }

            time_sell = get_time_by_dist(dist_sell);
            // 原料格空或 原料格满且生产时间小于到达时间
            if ( (Workspace::workspaces[target2Id]->resource_state[wk->type] == 0) ||
                (Workspace::workspaces[target2Id]->is_full_resource() &&              // 原料格满
                    Workspace::workspaces[target2Id]->remain_preduct_time != -1 &&       // 正在生产
                    (Workspace::workspaces[target2Id]->remain_preduct_time < (time_buy + time_sell + wait_product*2)) && // 到达的时候生产完成
                    Workspace::workspaces[target2Id]->preduct_state == 0) )            // 产品格必须为空（否则可能一直阻塞生产不出来）
            {
                //放宽卖点被选择条件后，该卖点没问题
            }else{
                continue;
            }

            // 增加判断游戏结束时候能不能卖出去，否则跳过
            if ((double)frameID + time_buy + time_sell + 300.0 > 12000.0){
                continue;
            }
            if ((double)frameID + time_buy + time_sell + 400.0 > 12000.0 && Workspace::workspaces[target2Id]->type == 7){
                continue;
            }
            if ((double)frameID + time_buy + time_sell + 500.0 > 12000.0 && wk->type == 7){
                continue;
            }
            bool is_near_enemy = 0;
            for(Robot* enemy_rot : enemy_robots){
                if(sqrt(pow(x - enemy_rot->x,2) + pow(y - enemy_rot->y, 2)) < r + 0.53 + 2.0){
                    is_near_enemy = 1;
                }
            }
            if((!Workspace::workspaces[target2Id]->is_safe || is_near_enemy) && ((double)frameID + time_buy + time_sell + 1000.0 > 12000.0)){
                continue;
            }
             //4 5 6缺原料的台优先考虑合成：
            double Q3 = 1.0;
            if(Workspace::workspaces[target2Id]->is_first()){
                Q3 = 0.84 * Q3;
            }


            //4 5 6被访问次数最少的工作台类型，加权值
            double Q1 = 1.0;
            if (num_of_seven && num_of_4 && num_of_5 && num_of_6 && frameID < 8760) {
                int min_visit = *min_element(Workspace::visit_times, Workspace::visit_times + 3);
                int max_visit = *max_element(Workspace::visit_times, Workspace::visit_times + 3);
                if (Workspace::visit_times[Workspace::workspaces[target2Id]->type - 4] == min_visit) {
                    if ((max_visit - min_visit) == 1) {
                        Q1 = 0.9;
                    }
                    if ((max_visit - min_visit) >= 2) {
                        Q1 = Q1 - 0.4 * (max_visit - min_visit - 1);
                    }
                    // cerr<<"man_visit="<<max_visit<<endl;
                    // cerr<<"min_visit="<<min_visit<<endl;
                }
            }
            

            // 用于7号台之间竞争456 
            double Q2 = 1.0;
            if (num_of_seven && num_of_4 && num_of_5 && num_of_6 && frameID < 8760) {
                if (Workspace::workspaces[target2Id]->type == 7) {
                    //原料格差多少个合成 
                    if (!Workspace::workspaces[target2Id]->is_full_reserved_and_resource()) {//防止gettime > remain_time就去送
                        //此时预定位还没置位，原料格会差 1 2 3个才能进入合成
                        if (Workspace::workspaces[target2Id]->num_of_ready_resouce_of_seven() == 3 &&
                            Workspace::workspaces[target2Id]->preduct_state == 0 &&
                            Workspace::workspaces[target2Id]->remain_preduct_time <= (time_buy + time_sell) &&
                            Workspace::workspaces[target2Id]->remain_preduct_time != -1) {
                            Q2 = 1.0;
                        }
                        else {
                            Q2 = 1.0 - 0.4 * Workspace::workspaces[target2Id]->num_of_ready_resouce_of_seven();
                            double q = (Workspace::workspaces[target2Id]->remain_preduct_time + 1.1) / 1000.0;//可调参数
                            Q2 = Q2 * (0.4 + q * 0.6);//尽量让7之间得竞争不影响到其他买卖点的cost

                        }
                    }
                }
            }

            // 计算和比较花销选择最小的情况
            double time_c = func(time_sell); // 时间系数
            double value = sell_buy[wk->type][1] - sell_buy[wk->type][0];
            double current_sum_dist = frameID >= 8760 ? (time_buy + time_sell)/(17.7*value) : (time_buy + time_sell);//TODO:后者考虑加上价值？防止不带走7的情况
            current_sum_dist *= Q1*Q2*Q3;
            //万不得已的情况下，会去不安全的地点卖。
            if(!wk->is_safe || !Workspace::workspaces[target2Id]->is_safe){
                current_sum_dist += 6666.0;
            }
            if(cost_sum_dist > current_sum_dist){
                cost_sum_dist = current_sum_dist;
                target1 = wk->id;
                target2 = target2Id;
                path_buy = temp_buy;
                path_sell = allpaths[wk->index_x][wk->index_y][target2Id][1];
            }
        }
        // 找不到其他地方卖，最后再考虑卖给9号工作台
        // 如果无暂存记录，且有9号
        if (target2 == -1 && nine_workspace_is)
        {
            for (Workspace *nine_wk : (Workspace::workspaces))
            {
                if(!nine_wk->is_safe && nine_wk->in_corner()>=2){
                    continue;
                }

                if(allpaths_valid[wk->index_x][wk->index_y][nine_wk->id][1]){
                        //可达
                }else{
                    continue;
                }
                if (nine_wk->type != 9)
                    continue;

                // int cnt = 0;
                // for (auto i : nine_wk->reserved) cnt += i.second;
                // if (cnt > 2) continue;

                double dist2 = allpaths_dis[wk->index_x][wk->index_y][nine_wk->id][1];
                time_sell_nine = get_time_by_dist(dist2);

                double cur_cost_nine = time_buy + time_sell_nine;
                if(!nine_wk->is_safe){
                    cur_cost_nine += 6666.0;
                }
                // 增加判断游戏结束时候能不能卖出去，否则跳过
                if ((double)frameID + time_buy + time_sell_nine + 300.0 > 12000.0)
                {
                    continue;
                }
                if ((double)frameID + time_buy + time_sell_nine + 400.0 > 12000.0 && (wk->type == 4 || wk->type == 5 || wk->type == 6 )){
                    continue;
                }
                if ((double)frameID + time_buy + time_sell_nine + 500.0 > 12000.0 && wk->type == 7){
                    continue;
                }
                bool is_near_enemy = 0;
                for(Robot* enemy_rot : enemy_robots){
                    if(sqrt(pow(x - enemy_rot->x,2) + pow(y - enemy_rot->y, 2)) < r + 0.53 + 2.0){
                        is_near_enemy = 1;
                    }
                }
                if((!nine_wk->is_safe || is_near_enemy) && ((double)frameID + time_buy + time_sell_nine + 1000.0 > 12000.0)){
                    continue;
                }

                if (cur_cost_nine < nine_cost)
                {
                    nine_cost = cur_cost_nine;
                    target1 = wk->id;
                    target2_nine = nine_wk->id;
                    path_buy = temp_buy;
                    path_sell_nine = allpaths[wk->index_x][wk->index_y][nine_wk->id][1];
                    record_time_buy = time_buy;
                    record_time_sell = time_sell;
                }
            }
        }
    }

    // // 处理目标点是否为远离中心的123买点
    // // 对于远离的4567买点，说明其附近一定也已经操作过了123456高频率买卖，可以执行
    // dist_buy = allpaths_dis[index_x][index_y][target1][0];
    // if((target2 != -1 || target2_nine != -1) && Workspace::workspaces[target1]->type <= 3 && // 目标点是123
    //     Workspace::workspaces[target1]->is_far_center && // 远离中心集群的点，非远离集群中心点去远处也能有大量任务
    //     dist_buy > avg_distance) // 自己也远离目标工作台
    //     {
    //         cerr<<dist_buy<<" "<<avg_distance<<endl;
    //         bool near_have = false; // 自己附近是否还有目标同类型的工作台
    //         for(auto wk:Workspace::workspaces_type_mp[Workspace::workspaces[target1]->type])
    //         {
    //             // 通过计算两个同类型工作台distance-dis_buy估计自己到此同类型工作台距离
    //             // 判断此距离小于平均距离一定范围，则尝试等待不去
    //             if(allpaths_valid[wk->index_x][wk->index_y][target1][0] && 
    //                 allpaths_dis[wk->index_x][wk->index_y][target1][0] - dist_buy < avg_distance*0.8)
    //             {
    //                 near_have = true;
    //                 break;
    //             }
    //         }
    //         if(near_have) 
    //         {
    //             far_continue_cnt++; // 停止的帧数
    //             if(far_continue_cnt < 100)
    //                 return; // 这里设置无任务跳过，就不去攻击敌人了
    //             else{
    //                 target2 = target1 = -1; // 置空任务，进入攻击
    //             }
    //         }
    //     }

    if(target2 != -1){
        task.push(make_pair(target1,path_buy));
        task.push(make_pair(target2,path_sell));

        //维护预定位数据结构
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target1]->type] = 1;


        //维护visit_times数组
        if (((Workspace::workspaces)[target2]->type >= 4) && (Workspace::workspaces)[target2]->type <= 6) {
            Workspace::visit_times[Workspace::workspaces[target2]->type - 4]++;
        }
    }
    else if (target2_nine != -1)
    {
        task.push(make_pair(target1,path_buy));
        task.push(make_pair(target2_nine,path_sell_nine));
        // 添加预定
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        // (Workspace::workspaces)[target2_nine]->reserved[(Workspace::workspaces)[target1]->type] = 1;
    }
    else{
        //找不到任务，临时攻击最近的敌方工作台
        Point attack_point;
        vector<int> temp;
        int reach = 0;
        for(int i=0; i<nums_of_workspaces; i++){
            if(workspace_is_reachable[i]) 
                reach++;
        }
        if(reach<=2){ // 图2中在敌方阵营
            if(select_eid_to_attack("ss", temp, attack_point, 0, id, 0)){
                if(stand_space_task(attack_point, 12000, 0, Point(index_x, index_y), 2000)){
                    nums_of_attack_robot ++;
                }
            }
        }
        else{
            if(select_eid_to_attack("ss", temp, attack_point, 0, id, 1)){
                stand_space_task(attack_point, 100, 0, Point(index_x, index_y), 2000);
            }
        }

    }
}


double Robot::func(double holdFrame)
{ // 计算时间系数,不考虑碰撞
    return (1 - sqrt(1 - pow(1 - holdFrame / 9000, 2))) * (1 - 0.8) + 0.8;
}

void Robot::buy()
{ // 机器人的买行为
    if (task.empty())
        return; // 为空队列就不需要买
    if (task.front().first < 0){
        return;
    }

    int next_workspace = task.front().first;                             // 当前目标工作台

    if (is_near_workspace != -1 && load_id == 0 &&                   // 附近有工作台 且 当前未携带物品
        is_near_workspace == next_workspace &&                       // 到达的这个工作台是目标工作台
        (Workspace::workspaces)[next_workspace]->preduct_state == 1) // 到达的这个工作台已生产出来
    {   
        // cerr<<id<<"do buy()..."<<endl;
        printf("buy %d\n", id);

        // cerr<<"buy time = "<<frameID<<endl;

        /* 任务出队 只能买了才能出队 */
        task.pop();

        /* 买了之后更新 目标工作台 产品格 被预定的状态为0 */
        Workspace::workspaces[next_workspace]->reserved[Workspace::workspaces[next_workspace]->type] = 0;
        /* 买了之后更新 目标工作台 产品状态为 空 */
        Workspace::workspaces[next_workspace]->preduct_state = 0;

        fighting_frame = 0;

        Workspace::workspaces[next_workspace]->attack_failed_times = 0;

        if(Workspace::workspaces[next_workspace]->is_safe == 0){
            Workspace::workspaces[next_workspace]->is_safe = 1;
            if(Workspace::workspaces[next_workspace]->type == 4){
                num_of_4 --;
            }
            if(Workspace::workspaces[next_workspace]->type == 5){
                num_of_5 --;
            }
            if(Workspace::workspaces[next_workspace]->type == 6){
                num_of_6 --;
            }
            if(Workspace::workspaces[next_workspace]->type == 7){
                num_of_seven --;
            }
        }
        
        

        /* 判断是否在工作台同一个位置，不在就重新a_star */
        if(index_x != Workspace::workspaces[next_workspace]->index_x ||
            index_y != Workspace::workspaces[next_workspace]->index_y) {
                // cerr<<" not in one site "<<endl;
                vector<Point> path;
                double dis;
                if(a_star(Point(index_x, index_y), task.front().first, 1, path, dis))
                {
                    task.front().second = path;
                }
                else{
                    // cerr<< " cant find "<<endl;
                }
            }
       
    }
    return;
}

void Robot::sell()
{ // 机器人的售出行为
    if (task.empty())
        return; // 为空队列就不需要卖

    if(task.front().first < 0){
        return;
    }

    int next_workspace = task.front().first; // 当前任务队列的队首，返回该工作台的id
    if (is_near_workspace != -1 && load_id > 0 &&
        is_near_workspace == next_workspace &&                                // 附近工作台是目标工作台
        (Workspace::workspaces)[next_workspace]->resource_state[load_id] == 0 // 目标工作台原料格为空
    )
    {
        // cerr<<id<<"do sell()..."<<endl;
        printf("sell %d\n", id);

        /* 任务出队 */
        task.pop();
        
        /* 卖了之后更新 目标工作台的 原料格(被机器人携带的产品) 被预定的状态 */
        Workspace::workspaces[next_workspace]->reserved[load_id] = 0;
        /* 卖了之后更新 目标工作台的 原料格(被机器人携带的产品) 装填的状态 */
        Workspace::workspaces[next_workspace]->resource_state[load_id] = 1;

        fighting_frame = 0;

        Workspace::workspaces[next_workspace]->attack_failed_times = 0;

        if(Workspace::workspaces[next_workspace]->is_safe == 0){
            Workspace::workspaces[next_workspace]->is_safe = 1;
            if(Workspace::workspaces[next_workspace]->type == 4){
                num_of_4 ++;
            }
            if(Workspace::workspaces[next_workspace]->type == 5){
                num_of_5 ++;
            }
            if(Workspace::workspaces[next_workspace]->type == 6){
                num_of_6 ++;
            }
            if(Workspace::workspaces[next_workspace]->type == 7){
                num_of_seven ++;
            }
        }
    }
}


/*-----------------------------------------------------------mov stable 最稳定----------------------------------*/
void Robot::move_stable()
{   
    // if(is_avoiding && task.size() && task.front().first >= 0){//TODO:
    //     cerr<<id<< " go  "<<Workspace::workspaces[task.front().first]->type<<endl;
    // }

    // if(near_face && near_wall && !is_stand_attacking && !is_stand_space_attacking && fighting_frame == 0){
    //     printf("forward %d %f\n", id, 7.);
    //     printf("rotate %d %f\n", id, PI);
    //     re_astar();
    //     return;
    // }

    if(is_fighting){
        
        // 边后退 边对准
        if(!task.empty() && !task.front().second.empty()){
            double tx = task.front().second[0].x;
            double ty = task.front().second[0].y;
            pair<double, double> f_vec(tx - x, ty - y);
            double f_vec_m = sqrt(f_vec.first * f_vec.first + f_vec.second * f_vec.second);
            double ft;
            if (f_vec_m < 1e-3) {
                ft = -rotation;
            }
            else {
                ft = atan2(f_vec.second, f_vec.first) - rotation;
            }
            if (ft > PI) {
                ft = ft - 2 * PI;
            }
            else if (ft < -PI) {
                ft = ft + 2 * PI;
            }
            printf("forward %d %f\n", id, -2.0);
            printf("rotate %d %f\n", id, 3. * ft);
        }
        else{
            printf("forward %d %f\n", id, -2.0);
            printf("rotate %d %f\n", id, 0);
        }        
        // cerr<<id<<" is fighting = "<<is_fighting<<endl;
        return;
    }

    if(!task.empty() && task.front().first >= 0){
        int target_wk = task.front().first;
        //解决边角工作台同时卖货时被自己人堵死的情况 : 增大礼让时间
        if(frameID % 200 < 80 && sqrt(pow(Workspace::workspaces[target_wk]->x - x,2)+ pow (Workspace::workspaces[target_wk]->y - y ,2)) < 3. ){//有友军在工作台，礼让
            int num_of_friendly = 0;
            for(Robot* rot : robots){
                if(rot == this){
                    continue;
                }
                if(abs(rot->index_x - Workspace::workspaces[target_wk]->index_x) + abs(rot->index_y - Workspace::workspaces[target_wk]->index_y) <= 2)
                {
                    num_of_friendly++;
                    break;
                }
            }
            if(num_of_friendly){
                printf("forward %d %f\n", id, -2.0);
                printf("rotate %d %f\n", id, 0);
                // cerr<<id<<" return 1.."<<endl;
                return;
            }
        } 
    }

    double temp_astar;
    // if(!task.empty() && !task.front().second.empty() && !is_avoiding){
    //     if(sqrt(pow(x- task.front().second[0].x, 2) + pow(y - task.front().second[0].y, 2)) > 2.5){
    //         a_star(Point(index_x,index_y),task.front().first,load_id,task.front().second,temp_astar);
    //     }
    // }

    double max_v ;
    if(color == 'r'){
        max_v = 7.0;
    }else{
        max_v = 6.0;
    }
    if(BFS_fail){
        // printf("forward %d %f\n", id, -2.0);
        // printf("rotate %d %f\n", id, 0);
        // return;
    }

    if (frameID % 300 == 0 || frameID == 1) {
        last_position = Point(index_x, index_y);
    }
    if (frameID % 300 > 50) {
        if (last_position.index_x == index_x && last_position.index_y == index_y
            && task.size() && task.front().first >= 0) {
                if(!task.empty() && !task.front().second.empty()){
                    if(near_face){
                        is_alive = 0;
                    }else{
                        if(!near_wall){
                            is_alive = 1;//每300帧检测机器人是否存活
                        }
                    }
                    is_avoiding = 0;
                    avoid_path.clear();
                    avoid_enemy_robots.clear();
                    avoid_robots.clear();
                    re_astar();
                }
                if(!is_avoiding){
                    printf("forward %d %f\n", id, max_v);
                    printf("rotate %d %f\n", id, PI);
                    return;
                }
        }
    }

    double theta, movcnt_theta, kpv_theta;
    double delete_boundary = 0.5;
    double cur_v = sqrt(v[0] * v[0] + v[1] * v[1]);

    if (cur_v > 5.) {
        delete_boundary = 1.;
    }
    else if (cur_v > 4.) {
        delete_boundary = 0.8;
    }


    if (is_avoiding) {
        if (avoid_path.size() > 1)
        {
            Point p1 = avoid_path[0];
            if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                // back_path.insert(back_path.begin(),avoid_path[0]);
                avoid_path.erase(avoid_path.begin());
            }
        }
    }
    else {
        if (task.empty() || task.front().second.empty())
        {
            printf("forward %d %f\n", id, 0.0);
            printf("rotate %d %f\n", id, 0.0);
            return;
        }
        if (task.front().second.size() > 1) {
            Point p1 = task.front().second[0];
            double delete_dot = (p1.x - x) * v[0] + (p1.y - y) * v[1];
            if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                task.front().second.erase(task.front().second.begin());
            }
        }
    }


    double targetx, targety;
    if (is_avoiding && avoid_path.size()) {
        targetx = avoid_path[0].x;
        targety = avoid_path[0].y;
    }
    else {
        if (task.empty() || task.front().second.empty())
        {
            printf("forward %d %f\n", id, 0.0);
            printf("rotate %d %f\n", id, 0.0);
            return;
        }
        targetx = task.front().second[0].x;
        targety = task.front().second[0].y;
    }

    bool stand_dir = false;

    if(is_stand_attacking && !task.empty() && !task.front().second.empty()
        && task.front().second.size()<=1 && task.front().first < 0 ){
            
        //守护距离
        Workspace* ws = real_workspace(task.front().first);
        vector<pair<double, pair<double, double>>> enemy_to_stand;
        for(Robot* rot : enemy_robots){
            double edis = sqrt(pow(ws->x - rot->x ,2) + pow(ws->y - rot->y, 2));
            enemy_to_stand.push_back(make_pair(edis, make_pair(rot->x, rot->y)));
        }
        sort(enemy_to_stand.begin(), enemy_to_stand.end());
        double now_enmey_position = 1e9;
        if(!enemy_to_stand.empty()){
            now_enmey_position = enemy_to_stand[0].first;
        }

        if(ws->in_corner() >= 2){
            //判断incorner的类型，偏移最终要占据的点的坐标TODO:
                if(!task.empty() && task.front().second.size() > 0){
                    Point target_p = task.front().second[task.front().second.size()-1];
                    if(graph[target_p.index_x + 2][target_p.index_y] == '#' || graph[target_p.index_x + 1][target_p.index_y] == '#'){//障碍物在工作台下方
                    target_p.y -= 0.25;
                    }
                    if(graph[target_p.index_x - 2][target_p.index_y] == '#' || graph[target_p.index_x - 1][target_p.index_y] == '#'){//障碍物在工作台上方
                        target_p.y += 0.25;
                    }
                    if(graph[target_p.index_x][target_p.index_y + 2] == '#' || graph[target_p.index_x][target_p.index_y + 1] == '#'){//障碍物在工作台右方
                        target_p.x += 0.25;
                    }
                    if(graph[target_p.index_x][target_p.index_y - 2] == '#' || graph[target_p.index_x][target_p.index_y - 1] == '#'){//障碍物在工作台左方
                        target_p.x -= 0.25;
                    }
                    targetx = target_p.x;
                    targety = target_p.y;
                }else{
                    cerr<<"mov deal with corner >= 2  error...";
                }      
        }
        else{
            if(now_enmey_position < 15.){
                double me_to_stand = sqrt(pow(ws->x - x , 2) + pow(ws->y - y , 2));
                stand_dir = true;
                targetx = enemy_to_stand[0].second.first;
                targety = enemy_to_stand[0].second.second;
                if(now_enmey_position < 2.7 && !enemy_to_stand.empty()){
                    stand_dir = false;
                    targetx = (enemy_to_stand[0].second.first + 
                            (ws->x - enemy_to_stand[0].second.first) / 2.);
                    targety = (enemy_to_stand[0].second.second + 
                            (ws->y - enemy_to_stand[0].second.second) / 2.);
                }
                if(now_enmey_position < 1.3 && !enemy_to_stand.empty()){
                    stand_dir = false;
                    targetx = (enemy_to_stand[0].second.first);
                    targety = (enemy_to_stand[0].second.second);
                }
            }   
        }
    }


    if(is_stand_space_attacking && !task.empty() && !task.front().second.empty() 
          && task.front().first < 0){
            vector<pair<double, pair<double, double>>> enemy_to_stand;
            int idx = (-task.front().first-100) % 100000 / 1000;
            int idy = (-task.front().first-10) / 100000;
            Point stand(idx, idy);
            //cerr<< idx<<" " <<idy<<endl;
            for(Robot* rot : enemy_robots){
                
                double edis = sqrt(pow(stand.x - rot->x ,2) + pow(stand.y - rot->y, 2));
                enemy_to_stand.push_back(make_pair(edis, make_pair(rot->x, rot->y)));
            }
            sort(enemy_to_stand.begin(), enemy_to_stand.end());
            bool near_face = false;
            Point near_enemy;
            for(Robot* rot : enemy_robots){
                bool outerloopcontinue = false;
                for(Robot* r : Robot::robots){
                    if(r->id == id) continue; 
                    if(sqrt(pow(rot->x - r->x, 2) + pow(rot->y - r->y, 2)) < 1.1){
                        outerloopcontinue = true;
                    }
                }
                if(outerloopcontinue) continue;
                double temp_dis = 2.5;
                if(mapID == 4) temp_dis = 2.5; // TODO: 图4特判
                if(sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < temp_dis){
                    near_face = true;
                    near_enemy = Point(rot->x, rot->y, 1);
                    break;
                }
            }
            double temp_dis = 7.7;
            if(mapID == 4) temp_dis = 7.7;
            if(near_face){
                targetx = near_enemy.x;
                targety = near_enemy.y;
            }
            else if(!enemy_to_stand.empty() && enemy_to_stand[0].first < temp_dis && task.front().second.size() < 2){
                targetx = enemy_to_stand[0].second.first;
                targety = enemy_to_stand[0].second.second;
            }
            else{
                double tmp;
                a_star(Point(index_x, index_y), stand, load_id, task.front().second, tmp);
            }
    }

            
    pair<double, double> dis_vec(targetx - x, targety - y);
    double dis_vec_m = sqrt(dis_vec.first * dis_vec.first + dis_vec.second * dis_vec.second);
    if (dis_vec_m < 1e-3) {
        theta = -rotation;
    }
    else {
        theta = atan2(dis_vec.second, dis_vec.first) - rotation;
    }
    if (theta > PI) {
        theta = theta - 2 * PI;
    }
    else if (theta < -PI) {
        theta = theta + 2 * PI;
    }



    bool will_in_wall = 0;
    int idx = 0, idy = 0;
    double vv = sqrt(v[0] * v[0] + v[1] * v[1]);
    double willx = x + vv * cos(rotation) * 10.0 / 50.0;
    double willy = y + vv * sin(rotation) * 10.0 / 50.0;
    get_index_by_coordinate(willx, willy, idx, idy);
    //cerr << idx << " " << idy << " " << graph[idx][idy] << endl;;

    if (graph[idx][idy] == '#')
        will_in_wall = 1;

    for(Robot* rot : Robot::robots){
        if(rot == this) continue;
        if(sqrt(pow(willx - rot->x,2) + pow(willy - rot->y, 2))< r + rot->r +0.1 && 
            (load_id || rot->load_id)){
            // cerr<<"robot is backing..."<<endl;
            will_in_wall = 1;
        }
    }

    double base_fs, fs, rs, theta_highspeed_boundary;

    if(color == 'b'){
        if (fabs(theta) > PI / 4. && load_id) {
            base_fs = 2.;
        }
        else if (fabs(theta) > PI / 6. && load_id) {
            base_fs = 7.;
        }
        else if (fabs(theta) > PI / 4. && !load_id) {
            base_fs = 6.;
        }
        else if (load_id) {
            base_fs = 17.;
        }
        else {
            base_fs = 23.;
        }
        rs = 7.0 * theta;
        fs = will_in_wall ? -2.00 : 1. / (fabs(rs) + 0.35) * base_fs;

        theta_highspeed_boundary = 0.15;
        if (!load_id) theta_highspeed_boundary = 0.2;
    }
    else{
        if (fabs(theta) > PI / 4. && load_id) {
            base_fs = 2.3;
        }
        else if (fabs(theta) > PI / 6. && load_id) {
            base_fs = 8.17;
        }
        else if (fabs(theta) > PI / 4. && !load_id) {
            base_fs = 7.;
        }
        else if (load_id) {
            base_fs = 14.;
        }
        else {
            base_fs = 27.;
        }
        rs = 11. * theta;
        fs = will_in_wall ? -2.00 : 1. / (fabs(rs) + 0.35) * base_fs;

        theta_highspeed_boundary = 0.1;
        if (!load_id) theta_highspeed_boundary = 0.15;
    }

    if (fabs(theta) < theta_highspeed_boundary) {
        rs = 0.;
        fs = 7.;
    }

    int path_size;

    if (is_avoiding) {
        path_size = avoid_path.size();
    }
    else {
        path_size = task.front().second.size();
    }
    if (dis_vec_m < 1.0 && path_size == 1 && 
    !is_stand_attacking && !is_stand_space_attacking && fighting_frame==0)
    {
        fs = 1.70 * dis_vec_m;
        if (dis_vec_m < 0.3 && is_avoiding) {
            fs = 0.0;
        }
    }

    // 占据进攻专用
    if(is_stand_attacking && stand_dir){
        rs = 7. * theta;
        fs = 0.;
    }

    printf("forward %d %f\n", id, fs);
    printf("rotate %d %f\n", id, rs);


    
}
/*-----------------------------------------------------------mov stable 最稳定----------------------------------*/


/*----------------------------------------------------------v1.0---------------------------------------------*/
double Robot::get_time_by_dist(double dist){
    // return dist / 4.7 * 50.0 + err*0.3;
    if(color == 'r'){
        return dist / 5.5 *50.0;
    }
    return dist / 5.0 * 50.0 ;

    //cerr << err << endl;
}
/*----------------------------------------------------------v1.0---------------------------------------------*/

bool Robot::low_priority(Robot* rot){
    //没任务的机器人优先级最高,没任务的都避让
    if(task.empty() || task.front().second.empty()) {
        return true;
    }
    if (rot->task.empty()) {
        return false;
    }
    // //防卡死 //TODO:已经用near_face判断，这段没用？
    // if(rot->is_alive == 0 && !rot->is_stand_attacking && !rot->is_stand_space_attacking){
    //     cerr<<"kasi"<<endl;
    //     return false;
    // }
    // if(!is_alive && !is_stand_attacking && !is_stand_space_attacking){//被卡死了
    //     return true;
    // }

    //特判：如果该机器人在我们的目标工作台上，且我们的目标工作台是3面墙的边角工作台，我们让他先走
    if(task.size() && task.front().first >= 0 && 
        point_near( Point(rot->index_x,rot->index_y),
        Point(Workspace::workspaces[task.front().first]->index_x,Workspace::workspaces[task.front().first]->index_y) )
        && Workspace::workspaces[task.front().first]->in_corner() >= 3)
    {
        // cerr<<"hello1"<<endl;
        return true;
    }
    //如果我们在目标机器人的工作台上，且该工作台三面环墙，我们先走
    if(rot->task.size() && rot->task.front().first >= 0 && 
        point_near( Point(index_x,index_y),
        Point(Workspace::workspaces[rot->task.front().first]->index_x,Workspace::workspaces[rot->task.front().first]->index_y) )
        && Workspace::workspaces[rot->task.front().first]->in_corner() >= 3)
    {
        return false;
    }

    //对方机器人在进攻被占据的工作台，不避让
    if(rot->fighting_frame > 0){
        return false;
    }

    if(rot->is_stand_space_attacking || rot->is_stand_attacking){
        return true;//TODO:
    }

    if(is_stand_space_attacking || is_stand_attacking){
        return false;
    }

    if(load_id < rot->load_id){
        return true;
    }else if (load_id == rot->load_id && (id < rot->id)){//都不带货或带货类型相同，id小的让路
        return true;
    }
    return false;
}
void Robot::friendly_conflict_detect(){
    // if(no_avoid_timing > 0){
    //     // cerr<<"skip friendly detect.."<<endl;
    //     return;
    // }

    //非攻击状态下，快到工作台的友军优先级高，不避让
    // if(!task.empty() && task.front().first >= 0 && load_id){
    //     int target_wk_id = task.front().first;
    //     if(sqrt(pow(Workspace::workspaces[target_wk_id]->x - x ,2)+pow(Workspace::workspaces[target_wk_id]->y - y,2)) < 5.){
    //         is_avoiding = 0;
    //         avoid_robots.clear();
    //         // no_avoid_timing = 2;//1有问题，下一帧--变为0，前面id小的机器人执行时检测不到
    //         return;
    //     }
    // }

    // if(!task.empty() && !task.front().second.empty() && task.front().first < 0){
    //     if((is_stand_attacking || is_stand_space_attacking) && task.front().second.size() < 10){
    //         return;
    //     }
    // }
    if(is_stand_space_attacking || is_stand_attacking){
        return;
    }
    //  第2阶段.维护avoid_robots的vector数组，检测该数组中机器人是否离开，删除元素
    if(!avoid_robots.empty()){
        // 维护路径寻路 backpath
        re_astar();
        //如果task为空，无任务要执行，就继续躲避其他机器人
        vector<Point> new_path;
        new_path = back_path; // =next_path - avoid_path
        if(!task.empty()){
            new_path.insert(new_path.end(),task.front().second.begin(),task.front().second.end());
        }
        // 判断是否还冲突
        for(int i = 0; i < avoid_robots.size();i++){
            // 首先比较优先级是否倒转就可以不避让
            if(!low_priority(avoid_robots[i])){
                avoid_robots.erase(avoid_robots.begin()+i); // 去除此机器人
                if(avoid_robots.empty()){ // 当无避让时，可以清空避让路径，回到正常任务队列开始运行
                    avoid_path.clear();
                    is_avoiding = 0;
                    next_path = new_path;
                    
                    //解决bug：被撞后由于点被删除，被墙卡住回不去目标点
                    re_astar();
                    //如果task为空，无任务要执行，就继续躲避其他机器人
                }
                continue;
            }

            bool remove = 1;

            //TODO:俩个机器人碰脸的情况？
            for(int j = 0; j <avoid_robots[i]->next_path.size();j++){ 
                if(j >= avoid_range){ 
                    break; 
                }
                bool outerloop_break = false;
                for(int k = 0; k < new_path.size(); k++){
                    if(k >= avoid_range){
                        break;
                    }
                    if(abs(j-k) > avoid_index_differ){ // 考虑冲突格与俩个机器人的距离
                        continue;
                    }
                    if(point_near(new_path[k],avoid_robots[i]->next_path[j])){
                        confilic_point[avoid_robots[i]] = avoid_robots[i]->next_path[j];
                        remove = 0;
                        outerloop_break = true;
                        break;
                    }
                }
                if (outerloop_break) break;
            }
            //如果当前要避让的机器人被敌方机器人贴脸，那我们就不做避让
            if(remove == 1 || (avoid_robots[i]->near_face && !is_stand_attacking && !is_stand_space_attacking)){ // 当前避让机器人已无冲突路径
                avoid_robots.erase(avoid_robots.begin()+i); // 去除此机器人
                if(avoid_robots.empty()){ // 当无避让时，可以清空避让路径，回到正常任务队列开始运行
                    avoid_path.clear();
                    is_avoiding = 0;
                    next_path = new_path;

                    //解决bug：被撞后由于点被删除，被墙卡住回不去目标点
                    re_astar(); 
                    //如果task为空，无任务要执行，就继续躲避其他机器人
                }
            }
        }
    }
    //  1.每帧做冲突检测  向avoid_robots的vector中添加元素
    for (Robot* rot : robots) {//遍历其他机器人，检测是否冲突
        if (rot->id == id) {
            continue;
        }
        if (!low_priority(rot)) {
            continue;
        }
        //如果已经在冲突机器人数组了，continue;
        if (find(avoid_robots.begin(), avoid_robots.end(), rot) != avoid_robots.end()) {
            continue;
        }
        // 如果自己已在对方避免碰撞数组，跳过
        if (find(rot->avoid_robots.begin(), rot->avoid_robots.end(), this) != rot->avoid_robots.end()) {
            continue;
        }
        for (int i = 0; i < rot->next_path.size(); i++) {
            if (i >= avoid_range) {
                break;
            }
            bool outerloop_break = false;
            for (int j = 0; j < next_path.size(); j++) {
                if (j >= avoid_range) {
                    break;
                }
                if (abs(j - i) > avoid_index_differ) continue;
                if (point_near(next_path[j], rot->next_path[i]) && !rot->near_face) {//路径存在邻格
                    //near_face 被敌方机器人贴脸，上面维护也要做这个判断
                    avoid_robots.push_back(rot);
                    // is_avoiding = 1;
                    confilic_point[rot] = rot->next_path[i];
                    outerloop_break = true;
                    break;
                }
            }
            if (outerloop_break) break;
        }
    }
}
void Robot::enemy_conflict_detect(){
    //1.维护需要避让和需要攻击的敌方机器人数组 
    if(avoid_enemy_robots.size()){
        // cerr<< id<< "clear enemy_rob +  a_star"<<endl;
        re_astar();
        avoid_enemy_robots.clear();
        avoid_path.clear();
        // attack_path.clear();
    }

    //2.对敌对机器人做碰撞检测
    // 如果快到工作台附近，不做避让

    if(!task.empty() && task.front().first >= 0 &&
        sqrt(pow(x - Workspace::workspaces[task.front().first]->x, 2) +
        pow(y - Workspace::workspaces[task.front().first]->y, 2)) < 5.)
    {
        return;
    }
    if(is_stand_attacking || is_stand_space_attacking){
        return;
    }
    for(auto enemy_rob : enemy_robots)
    {   
        if(enemy_rob->index_x == -1 && enemy_rob->index_y == -1)
            continue; // 此机器人并未出现在当前视野中

        if(!task.empty() && task.front().first >= 0 &&
            sqrt(pow(enemy_rob->x - Workspace::workspaces[task.front().first]->x, 2) +
            pow(enemy_rob->y - Workspace::workspaces[task.front().first]->y, 2)) < 2.) // 对于敌人在我目标工作台上，就不作避让了
            continue;

        for(int i = 0; i < next_path.size(); ++i)//出现在当前机器人接下来要走的路径上
        {
            if (i >= avoid_range * 2)
                break;
            if(point_near(next_path[i], Point(enemy_rob->index_x, enemy_rob->index_y)))
            {
                //如果不带货且敌方离得很近，干他
                if(load_id == 0 || is_stand_attacking || is_stand_space_attacking){//不带货，低优先级，攻击敌方
                    attack_enemy_robots.push_back(enemy_rob);
                }else if(color == 'r' && load_id){//带贵重货物，高优先级，避让敌方
                    avoid_enemy_robots.push_back(enemy_rob);
                }
                else if(color == 'b' && load_id == 7){
                    avoid_enemy_robots.push_back(enemy_rob);
                }
                break;
            }
        }
    }
}

void Robot::conflict_avoidance(){ 
    // if(no_avoid_timing > 0){
    //     // cerr<<"skip conflict avoidance.."<<endl;
    //     return;
    // }
    if(is_stand_attacking || is_stand_space_attacking){
        is_avoiding = 0;
        return;
    }
    //如果我在进攻，不避让
    if(fighting_frame > 0){
        is_avoiding = 0;
        return;
    }

    char graph_temp[width][width];
    //  3.避让处理逻辑 更新avoid_path
    if (!avoid_robots.empty() || !avoid_enemy_robots.empty()) 
    {
        // cerr<<id<<"robot handle conflic"<<endl;
        for (Robot *rot : avoid_robots)
        {
            // 先存graph
            for (int j = -1; j <= 1; j++)
            { // 冲突机器人本身变成障碍物
                for (int k = -1; k <= 1; k++)
                {
                    //冲突机器人周围一圈
                    //考虑边界越界：
                    if(in_range(rot->index_x + j,rot->index_y + k)){

                    }else{
                        continue;
                    }
                    graph_temp[rot->index_x + j][rot->index_y + k] = graph[rot->index_x + j][rot->index_y + k];
                }
            }
            for (int i = 0; i < rot->next_path.size(); i++)
            {
                if (i >= avoid_range)
                {
                    //只考虑next_path路径前十个点
                    break;
                }
                int x = rot->next_path[i].index_x;
                int y = rot->next_path[i].index_y;
                // 先存
                for (int j = -1; j <= 1; j++)
                { // 冲突机器人接下来路径上的点变为障碍物
                    for (int k = -1; k <= 1; k++)
                    {
                        //路径一圈变为障碍物
                        //考虑边界 越界：
                        if(in_range(x + j,y + k)){

                        }else{
                            continue;
                        }
                        graph_temp[x + j][y + k] = graph[x + j][y + k];
                    }
                }
            }
        }
        for(auto enemy_rob : avoid_enemy_robots)
        {
            for (int j = -1; j <= 1; j++)
                { // 冲突机器人本身变成障碍物
                for (int k = -1; k <= 1; k++)
                {
                    //冲突机器人周围一圈
                    //考虑边界越界：
                        if(in_range(enemy_rob->index_x + j,enemy_rob->index_y + k)){

                    }else{
                        continue;
                    }
                        graph_temp[enemy_rob->index_x + j][enemy_rob->index_y + k] = 
                                    graph[enemy_rob->index_x + j][enemy_rob->index_y + k];
                    }
                }
        }
        // 后graph更新障碍物区域
        for (Robot *rot : avoid_robots)
        {
            graph[rot->index_x][rot->index_y] = '#';
            for (int j = -1; j <= 1; j++)
            { // 冲突机器人本身及其邻格一圈变成障碍物
                for (int k = -1; k <= 1; k++)
                {   
                    bool outerloop_continue = false;
                    for (int m = -1; m <= 1; m++) {
                        for (int n = -1; n <= 1; n++) {
                            if (rot->index_x + j + m == index_x && rot->index_y + k + n == index_y) {
                                outerloop_continue = true;
                            }
                        }
                    }
                    if (outerloop_continue) continue;
                    if(in_range(rot->index_x + j,rot->index_y + k)){
                    graph[rot->index_x + j][rot->index_y + k] = '#';
                }
                    
                }
            }

            bool nei_flag = 0; 
            //判断俩个机器人是否邻格2圈
            for (int j = -2; j <= 2; j++)
            {
                for (int k = -2; k <= 2; k++)
                {
                    if(rot->index_x + j == index_x && rot->index_y + k == index_y){
                        nei_flag = 1;
                        break;
                    }
                }
            }
            if(nei_flag) // 表示两个机器人是挨着了已经
            {
                continue; // 挨着就不需要延伸路径了
            }

            for (int i = 0; i < rot->next_path.size(); i++)
            {
                // 最多只将障碍物加到冲突点或者最远扫描的10个格子
                if (confilic_point[rot].hash_f() == rot->next_path[i].hash_f()) {
                    break;
                }
                if (i >= avoid_range)
                {
                    break;
                }
                int x = rot->next_path[i].index_x;
                int y = rot->next_path[i].index_y;
                bool outerloop_break = false;
                for (int j = -1; j <= 1; j++) {
                    for (int k = -1; k <= 1; k++) {
                        if (x + j == index_x && y + k == index_y) {
                            outerloop_break = true;
                        }
                    }
                }
                if (outerloop_break) break;
                for (int j = -1; j <= 1; j++)
                {
                    for (int k = -1; k <= 1; k++)
                    {   
                        if (in_range(x + j, y + k))
                        graph[x + j][y + k] = '#';
                    }
                }
            }  
        }
        for(auto enemy_rob : avoid_enemy_robots) // 对敌对冲突机器人周围一圈加障碍物
        {
            for (int j = -1; j <= 1; j++)
            { // 冲突机器人本身变成障碍物
                for (int k = -1; k <= 1; k++)
                {
                    bool outerloop_continue = false;
                    for (int m = -1; m <= 1; m++) {
                        for (int n = -1; n <= 1; n++) {
                            if (enemy_rob->index_x + j + m == index_x && enemy_rob->index_y + k + n == index_y) {
                                outerloop_continue = true;
                            }
                        }
                    }
                    if (outerloop_continue) continue;
                    if(in_range(enemy_rob->index_x + j,enemy_rob->index_y + k)){
                        graph[enemy_rob->index_x + j][enemy_rob->index_y + k] = '#';
                    }
                    
                }
            }
            graph[enemy_rob->index_x][enemy_rob->index_y] = '#';
        }

        bool avoid_enemy_success = 0;
        bool is_our_attack_robot = 0;//要避让的友军中是否有己方正在攻击的机器人
        for(Robot* rot : avoid_robots){
            if(rot->id == id){
                continue;
            }
            if(rot->is_stand_attacking || rot->is_stand_space_attacking){
                is_our_attack_robot = 1;
                break;
            }
            //TODO:如果避让攻击机器人，绕开走
        }
        if(!avoid_enemy_robots.empty() || is_our_attack_robot){
            if(!task.empty() && task.front().first >= 0){
                double temp;
                if(a_star(Point(index_x,index_y),task.front().first,load_id,task.front().second,temp)){
                    //可以绕开走
                    // cerr<<task.front().second.size()<<endl;
                    is_avoiding = 0;
                    avoid_path.clear();
                    avoid_enemy_success = 1;
                    // cerr<<id<<" enemy avoid astar.."<<endl;//TODO:有问题，图2截图那里，为啥会a*成功
                }else{ 
                    //不避让,攻击？//TODO:
                    avoid_enemy_success = 0;
                    // cerr<<"avoid enemy fail.."<<endl;
                }
            }else{
                //干他
            }
        }
        if(!avoid_enemy_success && avoid_robots.size()){//直接a*没有成功，BFS避让自己人
            // cerr<<id<<" hello BFS.."<<endl;//TODO:这里进来了？
            // BFS找 合适的点 去躲避
            BFS_fail = 0;
            queue<Point> q;
            unordered_map<int,bool> visit;
            q.push(Point(index_x,index_y));
            bool find_des = 0; // bfs是否找到避让点 
            while(!q.empty()){
                Point cur = q.front();
                //判断该点是否满足条件
                bool cur_ok = 1;
                // 不带货 四周无障碍物就可选为为避让点
                if(!load_id){
                    for(int i = -1; i <= 1;i++){
                        for(int j = -1;j<=1;j++){
                            if(!in_range(cur.index_x + i,cur.index_y + j) || 
                                graph[cur.index_x + i][cur.index_y + j] == '#'){
                                cur_ok = 0;
                            }
                        }
                    }
                    // // 加严格选点条件
                    // if(!in_range(cur.index_x + 2,cur.index_y) ||
                    //     graph[cur.index_x + 2][cur.index_y] == '#'){
                    //     cur_ok = 0;
                    // }
                    // if(!in_range(cur.index_x - 2,cur.index_y) ||
                    //     graph[cur.index_x - 2][cur.index_y] == '#'){
                    //     cur_ok = 0;
                    // }
                    // if(!in_range(cur.index_x, cur.index_y + 2) ||
                    //     graph[cur.index_x][cur.index_y + 2] == '#'){
                    //     cur_ok = 0;
                    // }
                    // if(!in_range(cur.index_x,cur.index_y - 2) ||
                    //     graph[cur.index_x][cur.index_y - 2] == '#'){
                    //     cur_ok = 0;
                    // }
                }else{ // 带货 周围1圈+上下左右2距离 内不可以成为避让点
                    for(int i = -1; i <= 1;i++){
                        for(int j = -1;j<=1;j++){
                            if(!in_range(cur.index_x + i,cur.index_y + j) ||
                                graph[cur.index_x + i][cur.index_y + j] == '#'){
                                cur_ok = 0;
                            }
                        }
                    }
                    if(!in_range(cur.index_x + 2,cur.index_y) ||
                        graph[cur.index_x + 2][cur.index_y] == '#'){
                        cur_ok = 0;
                    }
                    if(!in_range(cur.index_x - 2,cur.index_y) ||
                        graph[cur.index_x - 2][cur.index_y] == '#'){
                        cur_ok = 0;
                    }
                    if(!in_range(cur.index_x, cur.index_y + 2) ||
                        graph[cur.index_x][cur.index_y + 2] == '#'){
                        cur_ok = 0;
                    }
                    if(!in_range(cur.index_x,cur.index_y - 2) ||
                        graph[cur.index_x][cur.index_y - 2] == '#'){
                        cur_ok = 0;
                    }
                }

                // 被冲突的机器人未来路径都设置无效点，不可以作为避让点，
                // 路径延申的点附近曼哈顿距离2以内不可选作避让终点

                // 不可以一直延申，如果要延申，可以延申20 30格子 ,不然过于严格 干扰其他
                for(Robot* rot : avoid_robots){
                    if(point_near(cur,Point(rot->index_x,rot->index_y))){
                            cur_ok = 0;
                    }
                    for(int i = 0;i < rot->next_path.size(); i++){
                        if(i >= 15){
                            break;
                        }
                        if(point_near(cur,rot->next_path[i])){
                            cur_ok = 0;
                        }
                    }
                }

                if(cur_ok == 1){ // 找到避让点，跳出去a_star寻路
                    BFS_fail_point = cur;
                    find_des = 1;
                    break;
                }

                q.pop();
                for(int i = -1; i <= 1;i++){
                    for(int j = -1;j<=1;j++){
                        Point temp_p(cur.index_x +i,cur.index_y+j);
                        int temp = temp_p.hash_f();
                        if(visit.count(temp)){//map中存在该元素
                            continue;
                        }
                        // 超出地图范围跳过
                        if(!in_range(temp_p.index_x, temp_p.index_y)) continue;
                        // 判断该点是否可以加入
                        int up,down,left,right;
                        if (in_danger(temp_p.index_x, temp_p.index_y, load_id, Point(-1,-1), up, down, left, right))
                            continue; 
                        // 加入队列
                        visit[temp] = 1;
                        q.push(temp_p);
                    }
                }
            }
            // a_star寻路
            if(find_des){
                double temp;//TODO:避让点不会很远，simple astar
                if(simple_a_star(Point(index_x,index_y),BFS_fail_point,load_id,avoid_path,temp)){
                    is_avoiding = 1;
                }else{
                    is_avoiding = 0;
                    // cerr << "BFS fail" << endl;
                    BFS_fail = 1;
                }
            }else{
                // cerr << "BFS fail" << endl;
                BFS_fail = 1;
            }
        }
        
        //还原地图
        for(Robot* rot : avoid_robots){
            //冲突机器人接下来路径上的点变为障碍物
            for(int i = 0; i < rot->next_path.size(); i ++){
                if(i >= avoid_range){
                    break;
                }
                int x = rot->next_path[i].index_x;
                int y = rot->next_path[i].index_y;
                for(int j = -1;j <= 1; j++){  
                    for(int k = -1;k <= 1; k++){
                        if(in_range(x + j,y + k)){

                        }else{
                            continue;
                        }
                        graph[x+j][y+k] = graph_temp[x+j][y+k];
                    }
                }
            }
            //冲突机器人本身变成障碍物
            int x = rot->index_x;
            int y = rot->index_y;
            for(int j = -1;j <= 1; j++){  
                for(int k = -1;k <= 1; k++){
                    if(in_range(rot->index_x + j,rot->index_y + k)){

                    }else{
                        continue;
                    }
                    graph[x+j][y+k] = graph_temp[x+j][y+k];
                }
            }
        }
        for(auto enemy_rob : avoid_enemy_robots)
        {
            for (int j = -1; j <= 1; j++)
            { // 冲突机器人本身变成障碍物
                for (int k = -1; k <= 1; k++)
                {
                    //冲突机器人周围一圈
                    //考虑边界越界：
                    if(in_range(enemy_rob->index_x + j,enemy_rob->index_y + k)){

                    }else{
                        continue;
                    }
                    graph[enemy_rob->index_x + j][enemy_rob->index_y + k] = 
                                graph_temp[enemy_rob->index_x + j][enemy_rob->index_y + k];
                }
            }
        }
        if(!avoid_enemy_success && avoid_robots.empty()){
            // cerr<<"hello re_astar avoid"<<endl;
            //直接a*没有成功，BFS避让自己人,如果没有自己人，即只有敌军还避不开，我们就直接干他S
            is_avoiding = 0;
            avoid_enemy_robots.clear();
            avoid_path.clear();
            re_astar();
        }

    }
    if(is_avoiding == 1){
        if(avoid_enemy_robots.empty()){
            avoid_times += (double)avoid_robots.size();
        }
        single_avoid_frames++;
        if(single_avoid_frames > 250){
            // no_avoid_timing = 100;//TODO:
            single_avoid_frames = 0;
        }
    }else{//TODO:置0太早，没有从根本上解决
        // single_avoid_frames = 0;
    }
}
void Robot::conflict_attack(){//TODO:
    // if(attack_enemy_robots.empty()){
    //     return;
    // }
    // for(Robot* attack_rot : attack_enemy_robots){
    //     double temp;
    //     Point target_position = Point(attack_rot->index_x,attack_rot->index_y);
    //     a_star(Point(index_x,index_y),target_position,load_id, attack_path, temp);
    //     break;
    // }
}

void Robot::update_all_robots_next_path(){
    if(avoid_path.empty() && avoid_enemy_robots.empty()){
        is_avoiding = 0;
    }
    else{
        is_avoiding = 1;
    }
    // 接下来的路径是由冲突转移路+返回路径+任务路径
    next_path = avoid_path;
    if(!task.empty() && !task.front().second.empty()){
        next_path.insert(next_path.end(), task.front().second.begin(),task.front().second.end());
    }
    if (!next_path.empty() && next_path[0].index_x != index_x && next_path[0].index_y != index_y) {
        next_path.insert(next_path.begin(), Point(index_x, index_y));
    }
}

void Robot::fight_invarder(){
    if(frameID % 50 != 0){//每50帧检测一次
        return;
    }
    // cerr<<"hello fight invarder"<<endl;
    for(Robot* rot : enemy_robots){
        if(sqrt(pow(rot->x - x , 2)+pow(rot->y - y , 2)) < r + 0.53 +0.2){//有机器人挨着
            is_avoiding = 0;
            is_fighting = 1;//接下来50帧后退蓄力
            // cerr<<"fighting = 1"<<endl;
            return;
        }
    }
    is_fighting = 0;
}
void Robot::detect_to_reseek(){//放碰撞处理后，mov前  //TODO:每一百帧做reseek，这个函数可以删除？
    if(task.empty()){
        return;
    }
    if(task.front().first < 0){
        return;
    }
    bool is_attacking = 0;//task的目标 first
    if(task.front().second.size() < 5)//表示快到目的工作台，开始检测
    {
        int target_id = task.front().first;
        if(fighting_frame > 0){
            fighting_frame++;
            is_avoiding = 0;
            if(fighting_frame >= 600){//TODO:生效了吗？
                Workspace::workspaces[target_id]->attack_failed_times++;
                 //攻击一段时间，若攻不下，置为is_attacking
                if(Workspace::workspaces[target_id]->is_safe == 1){
                    Workspace::workspaces[target_id]->is_safe = 0;
                    if(Workspace::workspaces[target_id]->type == 4){
                        num_of_4 --;
                    }
                    if(Workspace::workspaces[target_id]->type == 5){
                        num_of_5 --;
                    }
                    if(Workspace::workspaces[target_id]->type == 6){
                        num_of_6 --;
                    }
                    if(Workspace::workspaces[target_id]->type == 7){
                        num_of_seven --;
                    }
                }
                
                 is_attacking = 1;
                 is_fighting = 0;
            }else{
                fight_invarder();
            }
        }else{
        int num_of_enemy = 0;
        for(Robot* rot : enemy_robots){
            if(abs(rot->index_x - Workspace::workspaces[target_id]->index_x) + abs(rot->index_y - Workspace::workspaces[target_id]->index_y) <= 2)
            {
                num_of_enemy ++;
            }
        }
           
            //放宽过滤条件
            if((num_of_enemy >= 2 )  || (num_of_enemy && (color == 'r')) ||
            ((Workspace::workspaces[target_id]->in_corner()>=3) && (num_of_enemy)))
            {
                if(Workspace::workspaces[target_id]->is_safe == 1){
                    Workspace::workspaces[target_id]->is_safe = 0;
                    if(Workspace::workspaces[target_id]->type == 4){
                        num_of_4 --;
                    }
                    if(Workspace::workspaces[target_id]->type == 5){
                        num_of_5 --;
                    }
                    if(Workspace::workspaces[target_id]->type == 6){
                        num_of_6 --;
                    }
                    if(Workspace::workspaces[target_id]->type == 7){
                        num_of_seven --;
                    }
                }
                is_attacking = 1;
                fighting_frame = 0;
                is_fighting = 0;
            }else if(num_of_enemy)
            {
                fighting_frame++;
                if(fighting_frame >= 600){
                    //TODO:这里不会生效？
                    cerr<<"error detect..."<<endl;
                    //攻击一段时间，若攻不下，置为is_attacking
                    if(Workspace::workspaces[target_id]->is_safe == 1){
                        Workspace::workspaces[target_id]->is_safe = 0;
                        if(Workspace::workspaces[target_id]->type == 4){
                            num_of_4 --;
                        }
                        if(Workspace::workspaces[target_id]->type == 5){
                            num_of_5 --;
                        }
                        if(Workspace::workspaces[target_id]->type == 6){
                            num_of_6 --;
                        }
                        if(Workspace::workspaces[target_id]->type == 7){
                            num_of_seven --;
                        }
                    }   
                    is_attacking = 1;
                    is_fighting = 0;
                }else{
                    fight_invarder();
                }
            }else{
                //没有敌方机器人占据目标工作台
                // int num_of_friendly = 0;
                // for(Robot* rot : robots){
                //     if(rot == this){
                //         continue;
                //     }
                //     if(abs(rot->index_x - Workspace::workspaces[target_id]->index_x) + abs(rot->index_y - Workspace::workspaces[target_id]->index_y) <= 2)
                //     {
                //         // is_avoiding = 1;
                //         // no_avoid_timing = 0;
                //         // avoid_robots.push_back(rot);
                //         num_of_friendly++;
                //     }
                // }
                // if(num_of_friendly){
                //     fight_invarder();//让友方机器人先离开，防止墙角工作台被自己人卡死
                // }
            }
        }
        
        if(is_attacking){
            // Workspace::workspaces[target_id]->count_of_be_attacking ++;
            // if(Workspace::workspaces[target_id]->count_of_be_attacking >= 2){
            //     //关闭该工作台，设为不可达 
            //     Workspace::workspaces[target_id]->is_safe = 0;
                
            // }
            if(Workspace::workspaces[target_id]->is_safe == 1){
                Workspace::workspaces[target_id]->is_safe = 0;
                if(Workspace::workspaces[target_id]->type == 4){
                    num_of_4 --;
                }
                if(Workspace::workspaces[target_id]->type == 5){
                    num_of_5 --;
                }
                if(Workspace::workspaces[target_id]->type == 6){
                    num_of_6 --;
                }
                if(Workspace::workspaces[target_id]->type == 7){
                    num_of_seven --;
                }
            }
            Workspace::workspaces[target_id]->unsafe_frame = 1000;
            if(Workspace::workspaces[target_id]->type == 4){
                num_of_4 --;
            }
            if(Workspace::workspaces[target_id]->type == 5){
                num_of_5 --;
            }
            if(Workspace::workspaces[target_id]->type == 6){
                num_of_6 --;
            }
            if(Workspace::workspaces[target_id]->type == 7){
                num_of_seven --;
            }
            
            // Workspace::workspaces[target_id]->reserved[load_id] = 0;
            last_wk_id = target_id;//上一次添加入task队列的卖点
            reseek();
        }
    }else{
        fighting_frame = 0;//离开该目标点时置0
        is_fighting = 0;
        // if(frameID % 500 == 0){//每一百帧做一次reseek
        //     bool near_wall = 0;
        //     for(int i = -1; i <= 1; i++){
        //         for(int j = -1; j <= 1; j++){
        //             if(graph[index_x + i][index_y + j] == '#'){
        //                 near_wall = 1;
        //                 break;
        //             }
        //         }
        //     }
        //     if(!near_wall){//不贴墙时reseek，防掉帧
        //         reseek();
        //     }
        // }
    }
}
bool can_sell(int load_id , Workspace* wk){
    if(load_id == 1){
        if(wk -> type == 4 || wk->type ==5 || wk->type == 9){
            return true;
        }
    }
    if(load_id == 2){
        if(wk -> type == 6 || wk->type ==4 || wk->type == 9){
            return true;
        }
    }
    if(load_id == 3){
        if(wk -> type == 6 || wk->type ==5 || wk->type == 9){
            return true;
        }
    }
    if(load_id == 4 || load_id == 5 || load_id == 6){
        if(wk -> type == 7 || wk->type == 9){
            return true;
        }
    }
    if(load_id == 7){
        if(wk -> type == 8 || wk->type == 9){
            return true;
        }
    }
    return false;
}
void Robot::reseek(){
    if(task.size() && task.front().first < 0){
        return;
    }
    if(!load_id){//没带货
        if(task.size() == 2 && task.front().first >= 0){
            // cerr<<"reserved = 0.."<<endl;
            int wk_buy_id = task.front().first;
            Workspace::workspaces[wk_buy_id]->reserved[Workspace::workspaces[wk_buy_id]->type] = 0;
            task.pop();
            Workspace::workspaces[task.front().first]->reserved[Workspace::workspaces[wk_buy_id]->type] = 0;
            task.pop();
        }
        if(task.size()){
            cerr<<"reseek error... task.size != 2..."<<endl;
        }
        simple_seek();
    }else{
        //卖货
        double cost = 10000.0;
        int target = -1;
        vector<Point> path;
        for (auto wk : Workspace::workspaces){
            if(!wk->is_safe){
                continue;
            }
            if(allpaths_valid[Workspace::workspaces[last_wk_id]->index_x][Workspace::workspaces[last_wk_id]->index_y][wk->id][1]){
                //可达
            }else{
                continue;
            }
            if(wk->reserved[load_id] == 1)//该原料格被预定
            {
                continue;
            }
            if(!can_sell(load_id,wk)){
                continue;
            }
            
            int enemy_num = 0;
            for(auto rot : enemy_robots){
                if(abs(rot->index_x - wk->index_x) + abs(rot->index_y - wk->index_y) <= 2){
                    //工作台被敌方机器人占据
                    enemy_num ++;
                }
            }
            // if(enemy_num >= 2 || (enemy_num && (color == 'r') ||
            //     (enemy_num && (wk->in_danger_corner() >= 2))))
            //放宽条件
            if(enemy_num >= 2 || 
                (enemy_num && (wk->in_danger_corner() >= 2)))
            {
                continue;
            }
            // 原料格空或 原料格满且生产时间小于到达时间
            if  (wk->resource_state[load_id] == 0) 
            {
                //放宽卖点被选择条件后，该卖点没问题
            }else{
                continue;
            }
            double cur_cost = allpaths_dis[Workspace::workspaces[last_wk_id]->index_x][Workspace::workspaces[last_wk_id]->index_y][wk->id][1];
            if(cur_cost < cost){
                cost = cur_cost;
                target = wk->id;
            }
        }
        if(target == -1){
            //RESEEK找不到
            int num_of_enemy = 0;
            for(Robot* rot : enemy_robots){
                if(abs(rot->index_x - Workspace::workspaces[last_wk_id]->index_x) + abs(rot->index_y - Workspace::workspaces[last_wk_id]->index_y) <= 2)
                {
                    num_of_enemy ++;
                }
            }
            //绝对撞不开的
            if((num_of_enemy >= 2 )  ||
            ((Workspace::workspaces[last_wk_id]->in_corner()>=3) && (num_of_enemy)) 
                || (Workspace::workspaces[last_wk_id]->attack_failed_times >= 2))
                {
                    vector<int> temp;
                    Point attack_point;
                    if(select_eid_to_attack("ss", temp, attack_point, 0, id, 1)){
                        while(task.size()) task.pop();
                        if(stand_space_task(attack_point, 100, 1, Point(index_x, index_y), 2000)){
                            is_fighting = 0;
                            fighting_frame = 0;
                            Workspace::workspaces[last_wk_id]->reserved[load_id] = 0;
                        }else{
                            //失败后队列被清空，恢复
                            vector<Point> temp;
                            temp.push_back(Point(-1,-1));
                            task.push(make_pair(last_wk_id,temp));//TODO:这样做,path空有没有问题
                            re_astar();
                        }
                    }
                }
            
                fighting_frame = 2;
                is_fighting = 1;
                is_avoiding = 0;
                if(!task.empty() && task.front().first >= 0){
                    Workspace::workspaces[task.front().first]->reserved[load_id] = 1;
                }
                return;
        }
        while(!task.empty() && task.front().first >= 0){
            Workspace::workspaces[task.front().first]->reserved[load_id] = 0;
            task.pop();
        }
        a_star(Point(index_x,index_y),Point(Workspace::workspaces[target]->index_x,Workspace::workspaces[target]->index_y),load_id,path,cost, false);
        task.push(make_pair(target,path));
        is_fighting = 0;
        fighting_frame = 0;
        Workspace::workspaces[target]->reserved[load_id] = 1;
    }
}

bool Robot::re_astar(){
    double temp;
    //如果task为空,没任务，不需要重新做什么路径规划。
    //如果是攻击任务，不需要做re_astar
    if(!task.empty() && !task.front().second.empty() && task.front().first >= 0){
        bool is_load = load_id;
        if(allpaths_dis[index_x][index_y][task.front().first][is_load])
        { // 已有此路径了
            task.front().second = allpaths[index_x][index_y][task.front().first][is_load];
            return true;
        }
        else 
        {
            bool valid = a_star(Point(index_x,index_y), task.front().first,is_load, task.front().second, temp);
            if(valid){
                allpaths[index_x][index_y][task.front().first][is_load] = task.front().second;
                allpaths_dis[index_x][index_y][task.front().first][is_load] = temp;
                allpaths_valid[index_x][index_y][task.front().first][is_load] = true;
                return true;
            }
        }
    }
    return false;
}