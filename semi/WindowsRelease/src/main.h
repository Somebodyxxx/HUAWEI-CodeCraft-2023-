#ifndef MAIN_H
#define MAIN_H

#include <vector>

extern int close_do7_frame;//关闭优先合成7的策略，贪时间转换为贪价值
extern double reach_extern_time;//预估到达时可以等待的时间 time_buy + x > remaintime , time_buy + time_sell + 2*x > remaintime
extern double end_extern_time;//快到15000帧时，加的冗余
extern double total_average_speed;//全局平均速度，用于计算get_time_by_dist
extern int avoid_range;//避障时next_path向前的探测格子数
extern int avoid_conflict_gap;//每次碰撞时，发生冲突的格子，在俩个机器人索引上的差值

extern int num_of_4;
extern int num_of_5;
extern int num_of_6;

/* 全局帧交互id */
extern int frameID;

extern int num_of_seven;

/* 工作台数量 */
extern int nums_of_workspaces;

//根据场上原料是否充足，考虑贪时间还是贪价值？
extern bool optimize2;

/* 各产品的 购买价 和 出售价 表 */
/* 第一维：依次从1~7 7个物品，第二维： 第0个购买价，第1个出售价*/
/* 注意下标index 和 物品类型的编号是一致的，从1开始 */
const int sell_buy[8][2] = {
    {0, 0},
    {3000, 6000},
    {4400, 7600},
    {5800, 9200},
    {15400, 22500},
    {17200, 25000},
    {19200, 27500},
    {76000, 105000}
};

/* 存储各 workspace[i][j] 类型之间的卖关系 */
/* 0代表工作台 i的产品不售卖给j工作台，1代表可售卖 */
/* 注意下标index 和 物品类型的编号是一致的，从1开始 */
/* 注意123456都是不允许单独卖给9，这里作优化决策*/
extern double workspace_sell_ok[10][10];

/* 存储各个工作台i 到 工作台j 之间的距离 */
const int N = 100;
extern double workspaces_map[N][N];

/* 全局记录是否存在9号工作台 */
extern bool nine_workspace_is;

/* 存储各个工作台i 到 工作台j 之间的权重 */
extern double workspaces_map_weight[N][N];

/* 记录是哪张图 */
extern int mapID;

#endif
