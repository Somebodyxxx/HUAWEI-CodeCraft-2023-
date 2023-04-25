#include "robot.h"
#include "a_star.h"
#include "workspace.h"
#include "init.h"
#include <math.h>
#include<iostream>
using namespace std;
// 在攻击接口中使用, id<0为敌方工作台

bool is_num_in_vec(int num, vector<int>& vec){
    for(int i=0; i < vec.size(); i++){
        if(num == vec[i]) return true;
    }
    return false;
}

Workspace* real_workspace(int id) {
    return id < 0 ? Workspace::enemy_workspaces[-(id + 1)] : Workspace::workspaces[id];
}

// 将enemy_id加入task, enemy_id是正数, 该函数将(-enemy_id - 1, path)加入task队列
// hope_frame是期望占据的帧数, 到达帧数后该攻击任务pop重新seek
// 返回true表示进攻task添加成功
bool Robot::stand_attack_task(int enemy_id, bool is_load,Point sour_astar){
    int attack_id = -enemy_id - 1;
    vector<Point> attack_path;
    double tmp_dis;
    if(a_star(sour_astar, attack_id, is_load, attack_path, tmp_dis)){
        task.push(make_pair(attack_id, attack_path));
        is_stand_attacking = true;
        return true;
    }
    while(!task.empty()) task.pop();
    return false;
}

bool Robot::stand_space_task(Point des, int hope_frame, bool is_load, Point sour_astar, int hope_stand_time){
    int attack_id = -(des.index_x * 1000 + des.index_y * 100000 + 100);
    vector<Point> attack_path;
    double tmp_dis;
    if(a_star(sour_astar, des, is_load, attack_path, tmp_dis)){
        task.push(make_pair(attack_id, attack_path));
        attack_space_frame = hope_frame;
        stand_space_frame = hope_stand_time;
        is_stand_space_attacking = true;
        return true;
    }
    while(!task.empty()) task.pop();
    return false;
}

// 类似于buy sell, 在合适的时候将攻击task出队
void Robot::continue_attack(){
    if(task.empty() || (task.front().first >= 0)){
        //is_stand_attacking = is_around_attacking = false;
        return;
    }
    if(is_stand_attacking){

    }

    if(is_stand_space_attacking){
        attack_space_frame --;
        if(attack_space_frame <= 0){
            while(!task.empty()) task.pop();
            is_stand_space_attacking = false;
        }
        for(Robot* rot : enemy_robots){
            if(sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < 3.7){
                stand_space_frame = 2000;
                break;
            }
        }
        if(task.front().second.size() <= 2){
            stand_space_frame --;
        }
        if(stand_space_frame <= 0){
            while(!task.empty()) task.pop();
            is_stand_space_attacking = false;
            vector<int> tmp;
            Point attack_point;
            // 自动到其他热力值点占据
            if(load_id){
                if(select_eid_to_attack("ss", tmp, attack_point, 1, id, 0)&&
                    stand_space_task(attack_point, 12000, 1, Point(index_x, index_y), 2000)){}
                else if(select_eid_to_attack("ss", tmp, attack_point, 1, id, 0)&&
                    stand_space_task(attack_point, 12000, 1, Point(index_x, index_y), 2000)){}
                else{
                    stand_space_frame = 2000;
                    is_stand_space_attacking = true;
                }
            }
            else{
                if(select_eid_to_attack("ss", tmp, attack_point, 0, id, 0)&&
                stand_space_task(attack_point, 12000, 0, Point(index_x, index_y), 2000)){}
                else{
                    stand_space_frame = 2000;
                    is_stand_space_attacking = true;
                }
            }
        }
    }
}

vector<enemy_cluster> all_enemy_cluster;

// 初始化距离近的工作集群, 游走攻击
void init_all_enemy_cluster(){
    for(Workspace* i : Workspace::enemy_workspaces){
        vector<Workspace* > cur_cluster;
        cur_cluster.push_back(i);
        for(Workspace* j : Workspace::enemy_workspaces){
            if(enemy_allpaths_valid[i->index_x][i->index_y][j->id][0]
                && enemy_allpaths[i->index_x][i->index_y][j->id][0].size() < 10){
                    if(i == j) continue;
                    cur_cluster.push_back(j);
                }
        }
        if(cur_cluster.size() < 2) continue;
        double score = 0.;
        for(Workspace* c : cur_cluster){
            score += c->type;
        }
        //score /= (double)cur_cluster.size();
        all_enemy_cluster.push_back(enemy_cluster(cur_cluster, score));
    }
    sort(all_enemy_cluster.begin(), all_enemy_cluster.end());
}



vector<hot_stand_space> all_hot_stand_space;
void init_all_hot_stand_space(){

    // 敌方没有工作台, 没有热力值点
    if(Workspace::enemy_workspaces.size() < 2){
        all_hot_stand_space.clear();
        return;
    }

    for(int i=0; i<width; i++){
        for(int j=0; j<width; j++){
            if(is_alarm(i,j) || D[i][j] < 3) continue;
            hot_stand_space tmp(i, j);
            // if(tmp.cluster_score > 7.) continue;
            all_hot_stand_space.push_back(tmp);
        }
    }
    sort(all_hot_stand_space.begin(), all_hot_stand_space.end());

    // 不能离自己工作台近
    for(int i=0;i<all_hot_stand_space.size();i++){
        bool near_ws = false;
            for(Workspace* ws : Workspace::workspaces){
                if(abs(all_hot_stand_space[i].hss.index_x - ws->index_x) + 
                    abs(all_hot_stand_space[i].hss.index_y - ws->index_y) < 7){
                        near_ws = true;
                        break;
                    }
            }
        if(near_ws){
            all_hot_stand_space.erase(all_hot_stand_space.begin() + i);
            i--;
        }
    }

    // 最多只要7个热力值点
    for(int i=0; i < 7 && i<all_hot_stand_space.size(); i++){
        for(int j=i+1; j<all_hot_stand_space.size(); j++){
            int nearlast = abs(all_hot_stand_space[j].hss.index_x - all_hot_stand_space[i].hss.index_x)
                         + abs(all_hot_stand_space[j].hss.index_y - all_hot_stand_space[i].hss.index_y);
            if(nearlast < 9){
                all_hot_stand_space.erase(all_hot_stand_space.begin() + j);
                j--;
            }
        }
    }

    if(all_hot_stand_space.size() > 7){
        all_hot_stand_space.erase(all_hot_stand_space.begin() + 7, all_hot_stand_space.end());
    }

    for(int i = 0; i < all_hot_stand_space.size(); i++){
        for(Robot* rot : Robot::robots){
            vector<Point> p;
            double d;
            if(a_star(Point(rot->index_x, rot->index_y), all_hot_stand_space[i].hss, 0, p, d)){
                all_hot_stand_space[i].reachable_rot_id.push_back(rot->id);
            }
            if(a_star(Point(rot->index_x, rot->index_y), all_hot_stand_space[i].hss, 1, p, d)){
                all_hot_stand_space[i].reachable_rot_id_load.push_back(rot->id);
            }
        }
    }

    for(int i=0; i<all_hot_stand_space.size(); i++){
        for(Workspace* ws : Workspace::workspaces){
            if(ws->type != 1 && ws->type != 2) continue;
            vector<Point> p;
            double d;
            if(a_star(Point(ws->index_x, ws->index_y), all_hot_stand_space[i].hss, 1, p, d)){
                all_hot_stand_space[i].reachable_ws_12.push_back(ws);
            }
        }
    }

}


bool select_eid_to_attack(string attack_type, vector<int>& stand_ws_eid, Point& stand_space_point, 
                           bool is_load, int rot_id, bool temporary){
    if(all_hot_stand_space.empty()){
        stand_space_point = Point(-1, -1);
        return false;
    }
    if(!is_load){
        for(int i=0; i<all_hot_stand_space.size(); i++){
            if(is_num_in_vec(rot_id, all_hot_stand_space[i].reachable_rot_id)){
                stand_space_point = all_hot_stand_space[i].hss;
                if(!temporary){
                    hot_stand_space tmp = all_hot_stand_space[i];
                    all_hot_stand_space.erase(all_hot_stand_space.begin() + i);
                    all_hot_stand_space.insert(all_hot_stand_space.end(), tmp);
                }
                return true;
            }
        }
    }
    else{
        for(int i=0; i<all_hot_stand_space.size(); i++){
            if(is_num_in_vec(rot_id, all_hot_stand_space[i].reachable_rot_id_load)){
                stand_space_point = all_hot_stand_space[i].hss;
                if(!temporary){
                    hot_stand_space tmp = all_hot_stand_space[i];
                    all_hot_stand_space.erase(all_hot_stand_space.begin() + i);
                    all_hot_stand_space.insert(all_hot_stand_space.end(), tmp);
                }
                return true;
            }
        }
    }
    return false;
}   

