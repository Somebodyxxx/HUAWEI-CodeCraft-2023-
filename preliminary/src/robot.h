#ifndef ROBOT_H
#define ROBOT_H

#include "workspace.h"
#include "main.h"

#include <vector>
#include <string>
#include <queue>
#include <ctime>

using namespace std;

class Robot{
public:
    Robot(double x,double y):x(x),y(y){
        wait = 0;
        bool is_handle_collision = 1;//快结束的时候置0，关闭碰撞处理
    }//初始化生成机器人
    static int money;//所有机器人公用一个总资金
    bool is_handle_collision;

    /* 机器人类向量 */
    static vector<Robot*> robots;

    int id;//机器人编号
    double x,y;//机器人每帧实时坐标
    double rotation;//机器人当前朝向 -3.1415926~3.1415926;

    double v[2];//机器人实时线速度
    double w;//机器人实时角速度 rotate  : 正数表示顺时针旋转，负数表示逆时针旋转
    double r;//机器人当前的半径

    queue<int> task;//维护目标队列

    int is_near_workspace;//表示所处工作台id，-1表示不在工作台附近，范围【0，k-1】
    int load_id;//携带物品类型, 0 表示未携带物品。【1-7】
    int holdtime;//持有物品的时间/帧数
    int collidetimes;//持有时累计碰撞次数

    void init_frame(vector<string> v);//处理每帧的输入

    void seek();//机器人寻找下一 买——卖策略
    double func(double); // 计算价值系数
    void buy();//机器人的买行为
    void sell();//机器人的售出行为
    int wait;//0时代表 无需等待； 1时代表 需要在工作台等待卖出。

    void move();
    double get_time1(int target_ws_id);
    double get_time2(int target_ws_id1, int target_ws_id2, double t1);
    double get_time3(int target_ws_id1, int target_ws_id2, int target_ws_id3, double t2);

    double get_time1_by_dis(int target_ws_id);
    double get_time2_by_dis(int target_ws_id1, int target_ws_id2, double t1);
};



#endif