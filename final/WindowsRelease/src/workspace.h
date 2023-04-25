#ifndef WORKSPACE_H
#define WORKSPACE_H

#include "a_star.h"
#include "main.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <math.h>


using namespace std;

class Workspace{
public:
    int id;
    int type; // 工作台类型编号 [1-9]
    double x,y;
    
    int index_x;
    int index_y;

    char color;

    int attack_failed_times;//用于友方工作台，该台久攻不下，达到一定次数时，关闭此台
    bool is_safe;

    int unsafe_frame; // 上一次检测敌方机器人情况到的帧数，-1代表最近无检测到附近有敌方机器人

    //工作台类型为456，由原料格状态和remain_time更新影响因子
    static double impact_factor[3];
    static void init_impact_factor();
    //卖点为7时，根据原料格状态，调整456（为买点、卖点）的优先级，7缺少时
    void update_impact_factor();
    bool is_update;//同一个台是否已更新影响因子

    /* 工作台类向量 */
    static vector<Workspace*> workspaces;
    static vector<Workspace*> workspaces_type_mp[10]; // 不同类型工作台分别归类，方便取用 [type][]
    static vector<Workspace*> enemy_workspaces;
    static vector<Workspace*> enemy_workspaces_type_mp[10];

    vector<int> next_workspace; // 可达的工作台id，在初始化frame中设置

    static int visit_times[3];//统计type为4 5 6的工作台 为卖点的次数

    int remain_preduct_time;// -1表示没有生产 0表示产品格满而阻塞 ，正常表示生产剩余帧数 
    bool preduct_state;// 产品格状态,0表示没有，1表示生产完毕

    int resource_state_o; //原材料格状态,十进制表示情况
    unordered_map<int, bool> resource_state;//原材料格状态，map查找
    unordered_map<int, bool> reserved; // 原材料格 与 产品格 被机器人预定是否

    bool is_far_center; // 是否为远离集群的零散点


    Workspace(int type, double x,double y, int index_x, int index_y, char color);

    /**
     * @brief 在初始化帧输入完成并初始化所有workspace对象后，
     *       记录每一个工作台可售卖（可达）的工作台
     * @param workspaces 全部工作台的类集合
     */
    void init_next_workspace();

    /**
     * @brief 每一帧时更新当前工作台的原料产品状态
     * 
     * @param v 通过空格切分打包为向量后的输入帧信息，内容为string类型
     */
    void init_frame(vector<string> v);

    /**
     * @brief 返回原料格是否为满
     * 
     * @return true 原料格已满
     * @return false 原料未满
     */
    bool is_full_resource();
    bool is_full_reserved_and_resource();
    bool is_resouce_no_reserved();
    int num_of_ready_resouce_of_seven();

    bool is_first();//考虑456是否优先考虑送货，即456缺不缺另一半原料

    //判断工作台附近是否有障碍物
    bool is_near_barrier();

    bool is_corner_workspace();

    // 为0, 空旷的
    // 为1, 只有一个边角处于危险区
    // 为2, 可能是一面有墙(危险区), 也可能是对角有墙(危险区), 占据或者是被占据都有困难
    // 3以上 墙角 
    int in_danger_corner_score;
    int in_danger_corner();

    int in_corner();//工作台在墙角

    bool can_buy_to_attack; // 可以买了去攻击
};

// 在攻击接口中使用, id<0为敌方工作台
Workspace* real_workspace(int id);

// 在游走攻击中使用, 离得近的工作台集群
struct enemy_cluster
{   
    enemy_cluster(vector<Workspace* > c, double s){cluster = c; score = s;}

    vector<Workspace* > cluster;
    double score;       // 所有工作台的type之和, 分数越大优先级越高

    bool operator < (const enemy_cluster& e){
        return score > e.score;
    }
};
// 所有用于游走攻击的工作台集群, [0]的优先级最高
extern vector<enemy_cluster> all_enemy_cluster;
void init_all_enemy_cluster();


struct hot_stand_space
{
    Point hss;
    double D_score;  // D数组, 离障碍物的距离
    double cluster_score; // 距离最近集群的距离
    double hot_score;   // 热力值
    double score;
    vector<int> reachable_rot_id; // 不在构造函数, 在init
    vector<int> reachable_rot_id_load; // // 不在构造函数, 在init
    vector<Workspace* > reachable_ws_12;

    hot_stand_space(int idx, int idy){
        hss = Point(idx, idy);
        D_score = D[idx][idy];
        hot_score = enemy_hit_F[idx][idy];

        vector<double> cluster_dis;
        // 必须距离高分集群近
        for(int i=0; i < (int)all_enemy_cluster.size(); i++){
            double dis = 0.;
            bool have4567 = false;
            for(int j=0; j<all_enemy_cluster[i].cluster.size(); j++){
                double delta_x = hss.x - all_enemy_cluster[i].cluster[j]->x;
                double delta_y = hss.y - all_enemy_cluster[i].cluster[j]->y;
                dis += sqrt(pow(delta_x, 2)+pow(delta_y, 2));
                if(all_enemy_cluster[i].cluster[j]->type == 4 ||
                    all_enemy_cluster[i].cluster[j]->type == 5 ||
                    all_enemy_cluster[i].cluster[j]->type == 6 ||
                    all_enemy_cluster[i].cluster[j]->type == 7){
                        have4567 = true;
                    }
            }
            dis /= (double)all_enemy_cluster[i].cluster.size();
            if(have4567){
                cluster_dis.push_back(dis);
            }
        }
        sort(cluster_dis.begin(), cluster_dis.end());
        cluster_score = cluster_dis.empty() ? 1. : cluster_dis[0];

        score = (1. / (1. + 1.7 * D_score) * 1. / (1. + 1.1 * cluster_score) * hot_score);
    }

    bool operator < (const hot_stand_space& h){
        return score > h.score;
    }
};

extern vector<hot_stand_space> all_hot_stand_space;

void init_all_hot_stand_space();



// 
bool select_eid_to_attack(string attack_type, vector<int>& stand_ws_eid, Point& stand_space_point, bool is_load, int rot_id, bool temporary);

#endif