#ifndef WORKSPACE_H
#define WORKSPACE_H

#include <vector>
#include <string>
#include <unordered_map>

using namespace std;

class Workspace{
public:
    int id;
    int type; // 工作台类型编号 [1-9]
    double x,y;
    
    int index_x;
    int index_y;

    //工作台类型为456，由原料格状态和remain_time更新影响因子
    static double impact_factor[3];
    static void init_impact_factor();
    //卖点为7时，根据原料格状态，调整456（为买点、卖点）的优先级，7缺少时
    void update_impact_factor();
    bool is_update;//同一个台是否已更新影响因子

    /* 工作台类向量 */
    static vector<Workspace*> workspaces;

    vector<int> next_workspace; // 可达的工作台id，在初始化frame中设置

    static int visit_times[3];//统计type为4 5 6的工作台 为卖点的次数

    int remain_preduct_time;// -1表示没有生产 0表示产品格满而阻塞 ，正常表示生产剩余帧数 
    bool preduct_state;// 产品格状态,0表示没有，1表示生产完毕

    int resource_state_o; //原材料格状态,十进制表示情况
    unordered_map<int, bool> resource_state;//原材料格状态，map查找
    unordered_map<int, bool> reserved; // 原材料格 与 产品格 被机器人预定是否

    Workspace(int type, double x,double y, int index_x, int index_y);

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
    bool is_first();//考虑456是否优先考虑送货，即456缺不缺另一半原料

    //判断工作台附近是否有障碍物
    bool is_near_barrier();

    bool is_corner_workspace();

    int num_of_ready_resouce_of_seven();
};



#endif