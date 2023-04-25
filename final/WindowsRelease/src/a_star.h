#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

/* 图中工作台数量 */
const int Worspaces_Num = 55;

/* 图中点数量 */
const int width = 100;
const int point_nums = width * width;

/* 地图，#为障碍物，其余都为可通过区域 */
/* 注意地图存储方式是和输入保持一致的 */
extern char graph[width][width];

// 每个栅格距离最近的栅格距离
extern int D[width][width];                

void get_index_by_coordinate(double x, double y, int& index_x, int& index_y);


/* 点 */
class Point
{
public:
    /* 初始化 */
    Point()
    {
        index_x = -1;
        index_y = -1;
        x = y = 0.0;
    };
    Point(int a, int b)
    {
        index_x = a;
        index_y = b;
        x = index_y * 0.5 + 0.25;
        y = 50.0 - index_x * 0.5 - 0.25;
    }
    Point(double a, double b, bool is_double) {
        x = a;
        y = b;
        get_index_by_coordinate(a, b, index_x, index_y);
    }
    /* data */
    int index_x, index_y; // graph中格子的xy方向索引
    double x, y;          // 第0行第0列的中心坐标为:(0.25,49.75)。

    /* 比较符重载 */
    friend bool operator<(Point a, Point b)
    {
        if (a.index_x == b.index_x)
            return a.index_x < b.index_x;
        return a.index_y < b.index_y;
    }

    Point operator = (const Point& p) {
        index_x = p.index_x;
        index_y = p.index_y;
        x = p.x;
        y = p.y;
        return *this;
    }

    int hash_f() { return index_x*100+index_y; }
};

/* 路径点集 */
extern vector<Point> allpaths[width][width][Worspaces_Num][2];
/* 路径的距离长度，初始化为0.0 */
extern double allpaths_dis[width][width][Worspaces_Num][2];
// // 加上每个点在路径中的距离，用于冲突检测策略
// extern vector<double> allpaths_dis_vec[width][width][Worspaces_Num][2];
/* 路径是否可达,初始化为可达，只有a_star判断为不可达才会更新 */
extern bool allpaths_valid[width][width][Worspaces_Num][2];

/* 敌对地图规划信息 */
/* 路径点集 */
extern vector<Point> enemy_allpaths[width][width][Worspaces_Num][2];
/* 路径的距离长度，初始化为0.0 */
extern double enemy_allpaths_dis[width][width][Worspaces_Num][2];
/* 路径是否可达,初始化为可达，只有a_star判断为不可达才会更新 */
extern bool enemy_allpaths_valid[width][width][Worspaces_Num][2];
/* 敌对图热度 */
extern int enemy_hit_F[width][width];


/**
 * @brief Get the index by coordinate object
 *
 * @param x 真实坐标x
 * @param y 真实坐标y
 * @param index_x 栅格索引值x
 * @param index_y 栅格索引值y
 */
void get_index_by_coordinate(double x, double y, int &index_x, int &index_y);
bool is_alarm(int idx, int idy);
/**
 * @brief 返回当前格子是否有障碍物威胁，
 *          当前格子是障碍物，不能通过。
 *          当前格子左右均有障碍物或者上下均有障碍物，无法通过。
 *
 * @param index_x 当前格子在栅格图中的索引x
 * @param index_y 当前格子在栅格图中的索引y
 * @param is_load 当前线路是否有装载货物
 * @param des 终点
 * @param up 索引值上方栅格 是否有障碍物需要远离到 当前栅格下边界
 * @param down 索引值下方栅格 是否有障碍物需要远离到 当前栅格上边界
 * @param left 索引值左方栅格 是否有障碍物需要远离到 当前栅格右边界
 * @param right 索引值右方栅格 是否有障碍物需要远离到 当前栅格左边界
 * @return true 危险，不能通过
 * @return false 可以通过，无危险
 */
bool in_danger(int index_x, int index_y, bool is_load, const Point des, int &up, int &down, int &left, int &right);

/**
 * @brief 为地图上原点到目的点寻找最短路径，
 *          a_start算法可以寻找每个点八个方位作为相邻节点。
 *
 * @param SOUR 源节点
 * @param DES 目的节点
 * @param is_load 是否载货
 * @param path 路径
 * @param path_dis 路径距离
 * @return vector<Point> 返回路径的点集
 */
bool a_star(Point SOUR, int workspace_id, bool is_load, vector<Point> &path, double &path_dis);
bool a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis);
bool a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis, double is_optimaze);

/**
 * @brief 为避障时的重新a_star寻路，但限制其寻找的点数，防止掉帧
 * 
 * @param SOUR 
 * @param DES 
 * @param is_load 
 * @param path 
 * @param path_dis 
 * @return true 
 * @return false 
 */
bool simple_a_star(Point SOUR, int workspace_id, bool is_load, vector<Point> &path, double &path_dis);
bool simple_a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis);

/**
 * @brief 对某条路径执行远离障碍物操作
 * 
 * @param path 路径数组
 * @param is_load 
 */
void far_barrier(vector<Point>& path, bool is_load);

/**
 * @brief 对路径上45°转折点之间进行转折平滑为直线，对三个45°转折点间尝试消除中间点，重新规划路径
 * 
 * @param path 路径点
 * @param direct 上一个点->当前点的方向，size()=path-1,由于起始点无朝向
 * @param is_load 是否带货
 */
void optimaze_turning(vector<Point>& path, vector<int>& direct, bool is_load);

/**
 * @brief 初始化D障碍物距离数组
 * 
 */
void init_D();

#endif
