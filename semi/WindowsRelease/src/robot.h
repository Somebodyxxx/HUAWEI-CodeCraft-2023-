#ifndef ROBOT_H
#define ROBOT_H

#include "workspace.h"
#include "main.h"
#include "a_star.h"

#include <vector>
#include <string>
#include <queue>
#include <ctime>
#include <unordered_map>
#include <map>

using namespace std;

class Robot{
public:
    Robot(double x,double y):x(x),y(y){
        wait = 0;
        bool is_handle_collision = 1;//快结束的时候置0，关闭碰撞处理
        movcnt = direction = 0;
        time_err = time_err_record = 0.;
        time_err_cnt = 0.;
        from_where = Point(-1, -1);
        get_index_by_coordinate(x, y, index_x, index_y);
        BFS_fail = 0;
    }//初始化生成机器人
    static int money;//所有机器人公用一个总资金
    bool is_handle_collision;

    /* 机器人类向量 */
    static vector<Robot*> robots;

    int id;//机器人编号
    double x,y;//机器人每帧实时坐标
    double rotation;//机器人当前朝向 -3.1415926~3.1415926;

    int index_x,index_y;

    double v[2];//机器人实时线速度
    double w;//机器人实时角速度 rotate  : 正数表示顺时针旋转，负数表示逆时针旋转
    double r;//机器人当前的半径

    queue<pair<int,vector<Point> > > task;//维护目标队列

    unordered_map<int, bool> workspace_is_reachable; // 用workspace_id 判断是否可达该工作台

    int is_near_workspace;//表示所处工作台id，-1表示不在工作台附近，范围【0，k-1】
    int load_id;//携带物品类型, 0 表示未携带物品。【1-7】
    int holdtime;//持有物品的时间/帧数
    int collidetimes;//持有时累计碰撞次数

    void init_frame(vector<string> v);//处理每帧的输入

    void seek();//机器人寻找下一 买——卖策略
    void simple_seek(); // 最简易版本seek
    void seek_parallar();//并行策略 3+1

    double func(double); // 计算价值系数
    void buy();//机器人的买行为
    void sell();//机器人的售出行为
    int wait;//0时代表 无需等待； 1时代表 需要在工作台等待卖出。

    void move_2();
    void move_3();
    void move_stable();   
    void move_stable3(); 
     int movcnt, direction; // 0向前, 1等待, 2后退

    double get_time_by_dist(double dist);

    //记录每个机器人100帧前的位置 
    Point last_position;

    // 记录非碰撞下的每个机器人100帧前的位置堵死了 
    Point last_position2;

    //以下用于避障处理：
    void conflict_avoidance();
    //当前机器人需要躲避的机器人
    vector<Robot*> avoid_robots;
    // 当前机器人和需要躲避机器人之间的冲突点的映射
    map<Robot*, Point> confilic_point;
    //当前机器人需要做避让处理时的路径，每次move前先判断这个路径是否为空
    vector<Point> avoid_path;
    //避让之后回去的路，
    vector<Point> back_path;
    //记录现在move跑的是哪个path
    bool is_backing;
    //当前机器人接下来要走的路径
    vector<Point> next_path;
    //记录现在move跑的是哪一个path
    bool is_avoiding;

    Point avoid_Point;//BFS选择的避让点坐标


    // 维护next_path  vector数组，更新每个机器人接下来要走的路
    void update_all_robots_next_path();//在每帧初始化调用

    bool low_priority(Robot*);

    bool BFS_fail;

    
    //记录冲突时机器人所处的格子
    // Point back_target;


    double time_err, time_err_cnt, time_err_record;
    Point from_where;

    bool near_other_robot(Robot*);//用于工作台附近是否可以优先级反转

    void move_test();

};


#endif