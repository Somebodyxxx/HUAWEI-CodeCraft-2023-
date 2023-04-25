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
    Robot(double x, double y, char color):x(x), y(y), color(color){
        wait = 0;
        bool is_handle_collision = 1;//快结束的时候置0，关闭碰撞处理
        movcnt = direction = 0;
        get_index_by_coordinate(x, y, index_x, index_y);
        avoid_times = 0.0;
        single_avoid_frames = 0;
        if(color == 'b'){
            attack_threshold = 0.08;//0.1
        }
        if(color == 'r'){
            attack_threshold = 0.4;//0.4
        }
        nums_of_attack_threshold = 2;
        fighting_frame = 0;
        is_fighting = 0;
        no_avoid_timing = 0;

        is_alive = 1;
        last_position = Point(index_x,index_y);

        near_face = 0;
        near_wall = 0;

        is_stand_attacking = false;
        is_stand_space_attacking = false;
        is_avoiding = false;
    }//初始化生成机器人

    static int nums_of_attack_robot;
    static int money;//所有机器人公用一个总资金
    bool is_handle_collision;

    char color;
    bool is_alive;//当前机器人是否被撞死
    bool near_face;//当前机器人是否被敌方机器人贴脸
    bool near_wall;//当前机器人是否贴墙

    /* 机器人类向量 */
    static vector<Robot*> robots;
    static vector<Robot*> enemy_robots;

    int id;//机器人编号
    double x,y;//机器人每帧实时坐标
    double rotation;//机器人当前朝向 -3.1415926~3.1415926;

    int index_x,index_y;

    double v[2];//机器人实时线速度
    double w;//机器人实时角速度 rotate  : 正数表示顺时针旋转，负数表示逆时针旋转
    double r;//机器人当前的半径

    /* 雷达数据 */
    double radar[365];

    queue<pair<int,vector<Point> > > task;//维护目标队列

    unordered_map<int, bool> workspace_is_reachable; // 用workspace_id 判断是否可达该工作台
    unordered_map<int, bool> workspace_is_reachable_load; // 用workspace_id 判断是否带货可达该工作台
    vector<int> reachable_wk_id_v;

    int is_near_workspace;//表示所处工作台id，-1表示不在工作台附近，范围【0，k-1】
    int load_id;//携带物品类型, 0 表示未携带物品。【1-7】
    int holdtime;//持有物品的时间/帧数
    int collidetimes;//持有时累计碰撞次数

    void init_frame(vector<string> v);//处理每帧的输入

    
    void simple_seek(); // 最简易版本seek
    double func(double); // 计算价值系数
    void buy();//机器人的买行为
    void sell();//机器人的售出行为
    int wait;//0时代表 无需等待； 1时代表 需要在工作台等待卖出。

    void move_stable(); 
    int movcnt, direction; // 0向前, 1等待, 2后退

    double get_time_by_dist(double dist);

    //记录每个机器人100帧前的位置 
    Point last_position;

    // 记录非碰撞下的每个机器人100帧前的位置堵死了 
    Point last_position2;

    // 以下用于避障处理：
    void friendly_conflict_detect();
    void enemy_conflict_detect();
    void conflict_avoidance();
    void conflict_attack();
    // 当前机器人需要躲避的机器人
    vector<Robot*> avoid_robots;
    vector<Robot*> avoid_enemy_robots;
    vector<Robot*> attack_enemy_robots;

    // 当前机器人和需要躲避机器人之间的冲突点的映射
    map<Robot*, Point> confilic_point;
    // 当前机器人需要做避让处理时的路径，每次move前先判断这个路径是否为空
    vector<Point> avoid_path;
    // 避让之后回去的路，
    vector<Point> back_path;
    // vector<Point> attack_path;
    // 记录现在move跑的是哪个path
    bool is_backing;
    // 当前机器人接下来要走的路径
    vector<Point> next_path;
    // 记录现在move跑的是哪一个path
    bool is_avoiding;
    bool BFS_fail;
    Point BFS_fail_point;
    // 维护next_path  vector数组，更新每个机器人接下来要走的路
    void update_all_robots_next_path();//在每帧初始化调用

    bool low_priority(Robot*);

    bool re_astar();//针对当前任务队头重新做astar

    /*-----------------------进攻接口: 添加攻击目标务必使用以下接口------------------------*/
    static void stand_attack();
    bool assign_robot_to_attack(Workspace* wk);//判断可不可以派当前机器人去占据

    bool is_stand_attacking,  is_stand_space_attacking;

    int attack_space_frame; // 空地 热力值高的点
    int stand_space_frame;

    // 将enemy_id加入task, enemy_id是正数, 该函数将(-enemy_id - 1, path)加入task队列
    // hope_frame是期望占据的帧数, 到达帧数后该攻击任务pop重新seek
    // 返回true表示进攻task添加成功
    bool stand_attack_task(int enemy_id,bool is_load,Point sour_astar); 

    // 占据空地进攻
    bool stand_space_task(Point des, int hope_frame, bool is_load, Point sour_astar, int hope_stand_time);
    //vector<pair<hot_stand_space ,bool> > is_arrival_hss;
    void stand_space_attack();

    // 类似于buy sell, 在合适的时候将攻击task出队
    void continue_attack();

    void test_attack_seek();


    /*-----------------------防御接口: 被进攻时采用------------------------*/
    //机器人上一次的目标工作台，用于reseek
    int last_wk_id ;

    //统计该机器人避让的避让次数，每帧最多避让3次
    double avoid_times;
    //一次避让所花费的帧数，如果避让时间超过一定值，他将不再避让其他友方
    int single_avoid_frames;
    int no_avoid_timing;//当避让时间超过一定值时，设置一段时间内不避让其他人
    //根据得分难易，确定派机器人攻击时 avoidtimes\frameID需要达到的权值
    double attack_threshold;
    int nums_of_attack_threshold;

    void detect_to_reseek();//检测买点 卖点是否被攻击，考虑是否换工作台
    void reseek();//放弃当前目标，寻找其他目标点

    void fight_invarder();//当前任务的卖点工作台被占领，跟他对撞
    bool is_fighting;
    int fighting_frame;//当前机器人反抗入侵的持续帧数


};


#endif