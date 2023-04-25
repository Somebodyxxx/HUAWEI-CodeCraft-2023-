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


/* 地图，#为障碍物，其余都为可通过区域 */
/* 注意地图存储方式是和输入保持一致的 */
char graph[width][width];

/* index x, index y,到id j的路径点集,0代表不带货物，1代表带货物*/
/* 路径点集 */ 
vector<Point> allpaths[width][width][Worspaces_Num][2];
/* 路径的距离长度 */
double allpaths_dis[width][width][Worspaces_Num][2];
/* 路径是否可达 */
bool allpaths_valid[width][width][Worspaces_Num][2];

/* 敌对地图规划信息 */
/* 路径点集 */
vector<Point> enemy_allpaths[width][width][Worspaces_Num][2];
/* 路径的距离长度，初始化为0.0 */
double enemy_allpaths_dis[width][width][Worspaces_Num][2];
/* 路径是否可达,初始化为可达，只有a_star判断为不可达才会更新 */
bool enemy_allpaths_valid[width][width][Worspaces_Num][2];
/* 敌对图热度 */
int enemy_hit_F[width][width];

/* a_star中邻居的个方位位置以及距离, 上右下左 左上 右上 右下 左下 */
const int index_dx[8] = {-1, 0, 1,  1,  1,  0,  -1, -1};
const int index_dy[8] = { 1, 1, 1,  0, -1, -1,  -1,  0};
const double diagonal = 0.5 * sqrt(2);
const double dis[8] = {diagonal, 0.5, diagonal, 0.5, diagonal, 0.5, diagonal, 0.5};

/* 距离源节点cost 和<点信息, 上一节点的方位0~7对于index_dx\dy数组> */
typedef pair <double, Point> open_point;

/* a_star 全局变量*/
pair<int, int> LAST[width][width];  // <index_x,index_y>
bool CLOSE[width][width] = {0};     // 访问标记
double G[width][width];             // G(x) 到源点的欧式距离
int D[width][width];                // 每个栅格距离最近的栅格距离
int turn[width][width];             // 在一次a_star中该点积累当前路径的转折次数
int direct[width][width];           // 在一次a_star中该点积累当前点的路径朝向，
                                    //   -1代表初始值，0~7分别对应index_d数组中的方向



/**
 * @brief Get the index by coordinate object
 * 
 * @param x 真实坐标x
 * @param y 真实坐标y
 * @param index_x 栅格索引值x
 * @param index_y 栅格索引值y
 */
void get_index_by_coordinate(double x, double y, int &index_x, int &index_y) {
    index_y = floor(x / 0.5);
    index_x = 99 - floor(y / 0.5);
    index_y = index_y < 0 ? 0 : index_y;
    index_y = index_y > 99 ? 99 : index_y;
    index_x = index_x < 0 ? 0 : index_x;
    index_x = index_x > 99 ? 99 : index_x;
}

bool is_alarm(int idx, int idy) {
    if (idx < 0 || idx >= width || idy < 0 || idy >= width || graph[idx][idy] == '#') {
        return true;
    }
    return false;
}

/**
 * @brief 返回当前格子是否有障碍物威胁，
 *          当前格子是障碍物，不能通过。
 *          当前格子左右均有障碍物或者上下均有障碍物，无法通过。
 * @param index_x 当前格子在栅格图中的索引x,代表行索引
 * @param index_y 当前格子在栅格图中的索引y,代表列索引
 * @param is_load 当前线路是否有装载货物
 * @param des 终点
 * @param up 索引值上方栅格 是否有障碍物需要远离到 当前栅格下边界
 * @param down 索引值下方栅格 是否有障碍物需要远离到 当前栅格上边界
 * @param left 索引值左方栅格 是否有障碍物需要远离到 当前栅格右边界
 * @param right 索引值右方栅格 是否有障碍物需要远离到 当前栅格左边界
 * @return true 危险，不能通过
 * @return false 可以通过，无危险
 */
bool in_danger(int index_x, int index_y, bool is_load, Point des, int& upleft, int& upright, int& downleft, int& downright)
{
    if (is_alarm(index_x, index_y)) {
        return true;
    }
    int danger_dx[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
    int danger_dy[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };

    bool danger = false;

    int around[8] = { 0 };
    for (int i = 0; i < 8; i++) {
        around[i] = is_alarm(index_x + danger_dx[i], index_y + danger_dy[i]) ? 1 : 0;
    }
    // 1代表有障碍物
    int ul = 0, ur = 0, dl = 0, dr = 0;
    ul = around[0] || around[1] || around[3] ? 1 : 0;
    ur = around[1] || around[2] || around[4] ? 1 : 0;
    dl = around[3] || around[5] || around[6] ? 1 : 0;
    dr = around[4] || around[7] || around[6] ? 1 : 0;

    if (!is_load) {
        if (ul + ur + dl + dr <= 1) {
            danger = false;
        }
        else {
            danger = true;
            if (is_alarm(index_x, index_y + 1) && !is_alarm(index_x, index_y - 1) && is_alarm(index_x, index_y - 2)){
                if (!is_alarm(index_x + 1, index_y) && !is_alarm(index_x + 1, index_y - 1) &&
                    !is_alarm(index_x - 1, index_y) && !is_alarm(index_x - 1, index_y - 1)) {
                    danger = false;
                }
            }
            else if (is_alarm(index_x, index_y - 1) && !is_alarm(index_x, index_y + 1) && is_alarm(index_x, index_y + 2)) {
                if (!is_alarm(index_x + 1, index_y) && !is_alarm(index_x + 1, index_y + 1) &&
                    !is_alarm(index_x - 1, index_y) && !is_alarm(index_x - 1, index_y + 1)) {
                    danger = false;
                }
            }
            else if (is_alarm(index_x + 1, index_y) && !is_alarm(index_x - 1, index_y) && is_alarm(index_x - 2, index_y)) {
                if (!is_alarm(index_x, index_y + 1) && !is_alarm(index_x - 1, index_y + 1) &&
                    !is_alarm(index_x, index_y - 1) && !is_alarm(index_x - 1, index_y - 1)) {
                    danger = false;
                }
            }
            else if (is_alarm(index_x - 1, index_y) && !is_alarm(index_x + 1, index_y) && is_alarm(index_x + 2, index_y)) {
                if (!is_alarm(index_x, index_y + 1) && !is_alarm(index_x + 1, index_y + 1) &&
                    !is_alarm(index_x, index_y - 1) && !is_alarm(index_x + 1, index_y - 1)) {
                    danger = false;
                }
            }
        }
    }
    else {
        if (ul + ur + dl + dr <= 1) {
            danger = false;
            if (ul) { danger = is_alarm(index_x + 2, index_y) || is_alarm(index_x, index_y - 2) ? true : danger; }
            if (dl) { danger = is_alarm(index_x + 2, index_y) || is_alarm(index_x, index_y + 2) ? true : danger; }
            if (ur) { danger = is_alarm(index_x - 2, index_y) || is_alarm(index_x, index_y - 2) ? true : danger; }
            if (dr) { danger = is_alarm(index_x - 2, index_y) || is_alarm(index_x, index_y + 2) ? true : danger; }
        }
        else {
            danger = true;
        }

    }
    if (index_x == des.index_x && index_y == des.index_y) {
        danger = is_load && ul + ur + dl + dr >= 3 ? true : false; // >3必定是墙角 带货不能去墙角
    }
    if (!(index_x == des.index_x && index_y == des.index_y)) {
        upleft = ul;
        upright = ur;
        downleft = dl;
        downright = dr;
    }

    return danger;
}


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
bool a_star(Point SOUR, int workspace_id, bool is_load, vector<Point>& path, double& path_dis)
{   
    if(workspace_id <= -100) {
        int idy = (-workspace_id) / 100000;
        int idx = (-workspace_id - idy * 100000) / 1000;
        cerr << "err astar id"<<endl;
        return a_star(SOUR, Point(idx, idy), is_load, path, path_dis, false);
    }
    // workspace_id < 0为敌方工作台
    Workspace* ws = real_workspace(workspace_id);
    // 构建终点
    int idx, idy;
    get_index_by_coordinate(ws->x, ws->y,idx, idy);
    Point DES(idx, idy);

    return a_star(SOUR, DES, is_load, path, path_dis);
}

bool a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis)
{
    return a_star(SOUR, DES, is_load, path, path_dis, true);
}

bool a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis, double is_optimaze)
{
    memset(turn, 0x3f3f3f, sizeof turn);    // 初始化转折次数是很大的
    turn[SOUR.index_x][SOUR.index_y] = 0;
    memset(direct, -1, sizeof direct);      // 初始化每个点的路径朝向

    for(int i = 0; i < width; i++)
        for (int j = 0; j < width; j++) {
            G[i][j] = 1e9;
            LAST[i][j].first = LAST[i][j].second = -1;
            CLOSE[i][j] = 0;
        }
    // 初始化源点
    G[SOUR.index_x][SOUR.index_y] = 0.0;
    LAST[SOUR.index_x][SOUR.index_y].first = SOUR.index_x;
    LAST[SOUR.index_x][SOUR.index_y].second = SOUR.index_y;

    priority_queue<open_point, vector<open_point>, greater<open_point>> heap;

    // 插入源节点
    heap.push(make_pair(sqrt(pow(0.5 * (SOUR.index_x - DES.index_x), 2) + pow(0.5 * (SOUR.index_y - DES.index_y), 2)), SOUR));

    bool flag = false; // 是否找到路径

    int up = 0, down = 0, left = 0, right = 0;
    while (heap.size())
    {
        // 取cost最小的点出来
        auto min_elem = heap.top();
        heap.pop();

        Point now_p = min_elem.second;
        double distance = G[now_p.index_x][now_p.index_y]; // g(x)

        /* 可能访问过的就跳过 */
        if (CLOSE[now_p.index_x][now_p.index_y])
            continue;

        /* 标记为colse节点，已访问 */
        CLOSE[now_p.index_x][now_p.index_y] = true;

        /* 加入CLOSE的点是DES就结束搜索 */
        if (now_p.index_x == DES.index_x && now_p.index_y == DES.index_y)
        {
            flag = true;
            path_dis = distance;
            break;
        }

        // 当前点上下左右中心位置是否有障碍物
        bool up_center    = !(in_range(now_p.index_x,   now_p.index_y+1) && graph[now_p.index_x  ][now_p.index_y+1] != '#');
        bool down_center  = !(in_range(now_p.index_x,   now_p.index_y-1) && graph[now_p.index_x  ][now_p.index_y-1] != '#');
        bool left_center  = !(in_range(now_p.index_x-1, now_p.index_y)   && graph[now_p.index_x-1][now_p.index_y] != '#');
        bool right_center = !(in_range(now_p.index_x+1, now_p.index_y)   && graph[now_p.index_x+1][now_p.index_y] != '#');

        /* 将该点向前和左右垂直5个方向的邻居加入open集合 */
        vector<int> range_v;
        if(direct[now_p.index_x][now_p.index_y] == -1) 
            for(int i = 0; i < 8; i++) range_v.push_back(i);
        else {
            range_v.push_back((direct[now_p.index_x][now_p.index_y]+8-1)%8);
            range_v.push_back((direct[now_p.index_x][now_p.index_y]));
            range_v.push_back((direct[now_p.index_x][now_p.index_y]+8+1)%8);
        }
        for (int i:range_v)
        {
            // 邻居的栅格索引
            int new_index_x = now_p.index_x + index_dx[i];
            int new_index_y = now_p.index_y + index_dy[i];
            
            // 判断障碍物危险
            if (in_danger(new_index_x, new_index_y, is_load, DES, up, down, left, right))
                continue; 

            // 斜向运动时,不可以转交处斜向
            if(i == 0 && (up_center || left_center)) continue;
            if(i == 2 && (up_center || right_center)) continue;
            if(i == 4 && (down_center || right_center)) continue;
            if(i == 6 && (down_center || left_center)) continue;

            // 计算到当前点i方向拓展所需要的转弯转折次数
            int turn_cnt = 0;
            if (direct[now_p.index_x][now_p.index_y] != i)
                turn_cnt = turn[now_p.index_x][now_p.index_y] + 1;
            else
                turn_cnt = turn[now_p.index_x][now_p.index_y];

            // 计算当前节点的cost，（当前点到源点的dis + open点到当前点的dis） + h(x)open点欧氏距离到目的点
            double new_dist = distance + dis[i];
            // 距离障碍物影响参数，非载货的距离障碍物可以近一些
            double D_C = 3.0/(D[new_index_x][new_index_y]); 
            double new_cost = new_dist + sqrt(pow(0.5*(new_index_x - DES.index_x), 2) + pow(0.5*(new_index_y - DES.index_y), 2))
                                + D_C + 0.5 * turn_cnt;


            // 将邻居加入open集合
            if(G[new_index_x][new_index_y] >= new_dist){ // 新的上一节点是的距离源点更近
                if(G[new_index_x][new_index_y] == new_dist) {
                    // 路径的折点若更多就不需要更新路径
                    if(turn_cnt >= turn[new_index_x][new_index_y])
                        continue;
                }
                G[new_index_x][new_index_y] = new_dist;
                LAST[new_index_x][new_index_y] = {now_p.index_x, now_p.index_y};
                Point tmp(new_index_x, new_index_y);
                heap.push(make_pair(new_cost,tmp));
                direct[new_index_x][new_index_y] = i; // 记录当前点路径方向
                turn[new_index_x][new_index_y] = turn_cnt; // 更新当前点路径的转折次数
            }
        }
    }
    // 未找到路径
    if (!flag)
        return false;

    path.clear();
    vector<int> path_director; // 记录路径每一次的路径方向
    // 创建路径vector ，将目标点首先加入
    DES.x = DES.index_y * 0.5 + 0.25;
    DES.y = 50.0 - DES.index_x * 0.5 - 0.25;
    path.push_back(DES);

    // 循环遍历直到SOUR起点，起点的LAST为-1
    Point now_p = path[0];
    while (!(LAST[now_p.index_x][now_p.index_y].first == SOUR.index_x &&
            LAST[now_p.index_x][now_p.index_y].second==SOUR.index_y))
    {   
        Point tmp(LAST[now_p.index_x][now_p.index_y].first, LAST[now_p.index_x][now_p.index_y].second);
        tmp.x = tmp.index_y * 0.5 + 0.25;
        tmp.y = 50.0 - tmp.index_x * 0.5 - 0.25;

        // 记录各点的direct
        path_director.push_back(direct[now_p.index_x][now_p.index_y]);
		
        // 热力值更新
        // F_add(tmp.index_x, tmp.index_y);

        // 删去非障碍物附近的直线路径中间点
        // if(D[tmp.index_x][tmp.index_y] >= 3 && // 最大范围目前未6
        //     (direct[now_p.index_x][now_p.index_y] == direct[tmp.index_x][tmp.index_y]) &&
        //     !(LAST[tmp.index_x][tmp.index_y].first == SOUR.index_x &&
        //     LAST[tmp.index_x][tmp.index_y].second == SOUR.index_y)){ // 注意终点前不能跳过
        //     now_p = tmp;
        //     continue;
		// }
		
        path.push_back(tmp);
        now_p = tmp;
    }
    SOUR.x = SOUR.index_y * 0.5 + 0.25;
    SOUR.y = 50.0 - SOUR.index_x * 0.5 - 0.25;
    path.push_back(SOUR);
    reverse(path.begin(), path.end());
    reverse(path_director.begin(), path_director.end());

    // 对路径处理远离障碍物
    far_barrier(path, is_load);

    // 处理折线的a_star路径优化 TODO: 对不同图做不同处理
    // if(mapID != 3 && is_optimaze) optimaze_turning(path, path_director, is_load);

    return true;
}


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
bool simple_a_star(Point SOUR, int workspace_id, bool is_load, vector<Point>& path, double& path_dis)
{   
    if(workspace_id <= -100) {
        int idy = (-workspace_id) / 100000;
        int idx = (-workspace_id - idy * 100000) / 1000;
        cerr << "err astar id"<<endl;
        return a_star(SOUR, Point(idx, idy), is_load, path, path_dis, false);
    }
    // workspace_id < 0为敌方工作台
    Workspace* ws = real_workspace(workspace_id);
    // 构建终点
    int idx, idy;
    get_index_by_coordinate(ws->x, ws->y,idx, idy);
    Point DES(idx, idy);

    return simple_a_star(SOUR, DES, is_load, path, path_dis);
}
bool simple_a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis)
{
    memset(turn, 0x3f3f3f, sizeof turn);    // 初始化转折次数是很大的
    turn[SOUR.index_x][SOUR.index_y] = 0;
    memset(direct, -1, sizeof direct);      // 初始化每个点的路径朝向

    for(int i = 0; i < width; i++)
        for (int j = 0; j < width; j++) {
            G[i][j] = 1e9;
            LAST[i][j].first = LAST[i][j].second = -1;
            CLOSE[i][j] = 0;
        }
    // 初始化源点
    G[SOUR.index_x][SOUR.index_y] = 0.0;
    LAST[SOUR.index_x][SOUR.index_y].first = SOUR.index_x;
    LAST[SOUR.index_x][SOUR.index_y].second = SOUR.index_y;

    priority_queue<open_point, vector<open_point>, greater<open_point>> heap;

    // 插入源节点
    heap.push(make_pair(sqrt(pow(0.5 * (SOUR.index_x - DES.index_x), 2) + pow(0.5 * (SOUR.index_y - DES.index_y), 2)), SOUR));

    bool flag = false; // 是否找到路径

    int close_cnt = 0;
    // int dis_manh = (abs(SOUR.index_x - DES.index_x) + abs(SOUR.index_y - DES.index_y));
    // dis_manh*=dis_manh; 
    // dis_manh *= 2; // TODO: 控制这个阈值来可以扩展搜索时间

    int up = 0, down = 0, left = 0, right = 0;
    while (heap.size())
    {
        if(heap.size() + close_cnt > 2000) return false; // 最多2000格，大约5ms，直接break失败
        // 取cost最小的点出来
        auto min_elem = heap.top();
        heap.pop();

        Point now_p = min_elem.second;
        double distance = G[now_p.index_x][now_p.index_y]; // g(x)

        /* 可能访问过的就跳过 */
        if (CLOSE[now_p.index_x][now_p.index_y])
            continue;

        /* 标记为colse节点，已访问 */
        CLOSE[now_p.index_x][now_p.index_y] = true;
        close_cnt++; // 记录已加入访问的点

        /* 加入CLOSE的点是DES就结束搜索 */
        if (now_p.index_x == DES.index_x && now_p.index_y == DES.index_y)
        {
            flag = true;
            path_dis = distance;
            break;
        }

        // 当前点上下左右中心位置是否有障碍物
        bool up_center    = !(in_range(now_p.index_x,   now_p.index_y+1) && graph[now_p.index_x  ][now_p.index_y+1] != '#');
        bool down_center  = !(in_range(now_p.index_x,   now_p.index_y-1) && graph[now_p.index_x  ][now_p.index_y-1] != '#');
        bool left_center  = !(in_range(now_p.index_x-1, now_p.index_y)   && graph[now_p.index_x-1][now_p.index_y] != '#');
        bool right_center = !(in_range(now_p.index_x+1, now_p.index_y)   && graph[now_p.index_x+1][now_p.index_y] != '#');

        /* 将该点向前3个方向的邻居加入open集合 */
        vector<int> range_v;
        if(direct[now_p.index_x][now_p.index_y] == -1) 
            for(int i = 0; i < 8; i++) range_v.push_back(i);
        else {
            range_v.push_back((direct[now_p.index_x][now_p.index_y]+8-1)%8);
            range_v.push_back((direct[now_p.index_x][now_p.index_y]));
            range_v.push_back((direct[now_p.index_x][now_p.index_y]+8+1)%8);
        }
        for (int i:range_v)
        {
            // 邻居的栅格索引
            int new_index_x = now_p.index_x + index_dx[i];
            int new_index_y = now_p.index_y + index_dy[i];
            // 判断障碍物危险
            if (in_danger(new_index_x, new_index_y, is_load, DES, up, down, left, right))
                continue; 

            // 斜向运动时,不可以转交处斜向
            if(i == 0 && (up_center || left_center)) continue;
            if(i == 2 && (up_center || right_center)) continue;
            if(i == 4 && (down_center || right_center)) continue;
            if(i == 6 && (down_center || left_center)) continue;

            // 计算到当前点i方向拓展所需要的转弯转折次数
            int turn_cnt = 0;
            if (direct[now_p.index_x][now_p.index_y] != i)
                turn_cnt = turn[now_p.index_x][now_p.index_y] + 1;
            else
                turn_cnt = turn[now_p.index_x][now_p.index_y];

            // 计算当前节点的cost，（当前点到源点的dis + open点到当前点的dis） + h(x)open点欧氏距离到目的点
            double new_dist = distance + dis[i];
            // 距离障碍物影响参数，非载货的距离障碍物可以近一些
            double D_C = 3.0/(D[new_index_x][new_index_y]); 
            double new_cost = new_dist + sqrt(pow(0.5*(new_index_x - DES.index_x), 2) + pow(0.5*(new_index_y - DES.index_y), 2))
                                + D_C + 0.5 * turn_cnt;


            // 将邻居加入open集合
            if(G[new_index_x][new_index_y] >= new_dist){ // 新的上一节点是的距离源点更近
                if(G[new_index_x][new_index_y] == new_dist) {
                    // 路径的折点若更多就不需要更新路径
                    if(turn_cnt >= turn[new_index_x][new_index_y])
                        continue;
                }
                G[new_index_x][new_index_y] = new_dist;
                LAST[new_index_x][new_index_y] = {now_p.index_x, now_p.index_y};
                Point tmp(new_index_x, new_index_y);
                heap.push(make_pair(new_cost,tmp));
                direct[new_index_x][new_index_y] = i; // 记录当前点路径方向
                turn[new_index_x][new_index_y] = turn_cnt; // 更新当前点路径的转折次数
            }
        }
    }
    // 未找到路径
    if (!flag)
        return false;

    path.clear();
    vector<int> path_director; // 记录路径每一次的路径方向
    // 创建路径vector ，将目标点首先加入
    DES.x = DES.index_y * 0.5 + 0.25;
    DES.y = 50.0 - DES.index_x * 0.5 - 0.25;
    path.push_back(DES);

    // 循环遍历直到SOUR起点，起点的LAST为-1
    Point now_p = path[0];
    while (!(LAST[now_p.index_x][now_p.index_y].first == SOUR.index_x &&
            LAST[now_p.index_x][now_p.index_y].second==SOUR.index_y))
    {   
        Point tmp(LAST[now_p.index_x][now_p.index_y].first, LAST[now_p.index_x][now_p.index_y].second);
        tmp.x = tmp.index_y * 0.5 + 0.25;
        tmp.y = 50.0 - tmp.index_x * 0.5 - 0.25;

        // 记录各点的direct
        path_director.push_back(direct[now_p.index_x][now_p.index_y]);
		
        // 热力值更新
        // F_add(tmp.index_x, tmp.index_y);

        // 删去非障碍物附近的直线路径中间点
        // if(D[tmp.index_x][tmp.index_y] >= 3 && // 最大范围目前未6
        //     (direct[now_p.index_x][now_p.index_y] == direct[tmp.index_x][tmp.index_y]) &&
        //     !(LAST[tmp.index_x][tmp.index_y].first == SOUR.index_x &&
        //     LAST[tmp.index_x][tmp.index_y].second == SOUR.index_y)){ // 注意终点前不能跳过
        //     now_p = tmp;
        //     continue;
		// }
		
        path.push_back(tmp);
        now_p = tmp;
    }
    SOUR.x = SOUR.index_y * 0.5 + 0.25;
    SOUR.y = 50.0 - SOUR.index_x * 0.5 - 0.25;
    path.push_back(SOUR);
    reverse(path.begin(), path.end());
    reverse(path_director.begin(), path_director.end());

    // 对路径处理远离障碍物
    far_barrier(path, is_load);

    return true;
}



/**
 * @brief 初始化D数组，每个点距离最近障碍物栅格的距离，最大为5
 * 
 */
// 处理距离为len处一圈点
void square_search(int i, int j, int len) {
    for(int k = -len; k <= len; k++){ // 上方len处边len*2+1个栅格距离为len
        if(!in_range(i+k, j+len)) continue;
        D[i+k][j+len] = min(D[i+k][j+len], len); // 取最近距离障碍
    }
    for(int k = -len; k <= len; k++){
        if(!in_range(i+k, j-len)) continue;
        D[i+k][j-len] = min(D[i+k][j-len], len); // 取最近距离障碍
    }
    for(int k = -len; k <= len; k++){
        if(!in_range(i+len, j+k)) continue;
        D[i+len][j+k] = min(D[i+len][j+k], len); // 取最近距离障碍
    }
    for(int k = -len; k <= len; k++){
        if(!in_range(i-len, j+k)) continue;
        D[i-len][j+k] = min(D[i-len][j+k], len); // 取最近距离障碍
    }
}
void init_D() 
{
    int detect_range = 6; // 检测范围

    for(int i = 0; i < width; i++)
        for(int j = 0; j < width; j++)
            D[i][j] = detect_range;

    for(int i = 0; i < width; i++) {
        for(int j = 0; j < width; j++){
            if(graph[i][j] == '#'){
                D[i][j] = 0;
                for(int k = 1; k < detect_range; k++)
                    square_search(i,j,k);
            }
        }
    }
}


/**
 * @brief 对路径上45°转折点之间进行转折平滑为直线，对三个45°转折点间尝试消除中间点，重新规划路径
 * 
 * @param path 路径点
 * @param direct 上一个点->当前点的方向，size()=path-1,由于起始点无朝向
 * @param is_load 是否带货
 */
void optimaze_turning(vector<Point>& path, vector<int>& direct, bool is_load)
{
    vector<Point> new_path;

    if(path.size() == 0 || direct.size() == 0) return ;
    vector<int> turning_index; // 记录转折点的在路径中的index
    turning_index.push_back(0);
    // 首先记录所有的转折点
    for(int i = 1; i < direct.size(); i++)
    {
        if(direct[i] != direct[i-1]) turning_index.push_back(i+1);
    }
    turning_index.push_back(path.size()-1);

    /**
     * - 每3个点之间进行消除中间转折点优化，只对两个边至少一边大于5长度的才做优化
     * - 从第一个定点开始向中点(第二个点)移动1点每次进行尝试对第三个点连线
     */
    for(int i = 2; i < turning_index.size(); i++)
    {
        if(turning_index[i]-turning_index[i-1] < 3 || turning_index[i-1]-turning_index[i-2] < 5)
        {
            for(int j = turning_index[i-2]; j < turning_index[i-1]; j++) // i++前将老路径添加到新路径
                new_path.push_back(path[j]);
            if(i == turning_index.size()-1) // 若i要break，老路径最后一个转折点之后的路径也要添加到新路径
            {
                for(int j = turning_index[i-1]; j <= turning_index[i]; j++)
                    new_path.push_back(path[j]);
            }
            continue;
        }

        Point des = path[turning_index[i]-2];
        // 从第一个点开始一直递增到中间点前5个位置，试图通过此点和第三个点连线看能不能通过
        bool find_path = false;
        for(int j = turning_index[i-2]+3; j <= turning_index[i-1] - 5; j++) // 从第3个点开始防止拐弯过大
        {
            Point sour = path[j];
            int point_cnt = sqrt(pow(sour.x - des.x, 2) + pow(sour.y - des.y, 2)) / 0.65;
            vector<Point> insert_path; // 需要新插入的点集
            // 向目标延伸的每个点x y方向增量
            double incre_x = (des.x - sour.x)/point_cnt;
            double incre_y = (des.y - sour.y)/point_cnt;
            Point now_p = sour;
            bool flag = true; // 该连线中间无障碍物
            for(int k = 0; k < point_cnt; k++)
            {
                Point temp_p(now_p.x + incre_x, now_p.y + incre_y, true);
                
                int upleft = 0, upright = 0, downleft = 0, downright = 0;
                if(in_danger(temp_p.index_x, temp_p.index_y, is_load, Point(-1,-1), upleft, upright, downleft, downright))
                {
                    flag = false; // 此路径附近有障碍物威胁，无法成功
                    break;
                }
                if(upleft+upright+downleft+downright > 0) 
                {
                    flag = false; // 此路径附近有障碍物威胁，无法成功
                    break;
                }
                insert_path.push_back(temp_p);
                now_p = temp_p;
            }
            if(flag) // 找到这样一条路可以替换原来的转折路径
            {
                for(int k = turning_index[i-2]; k <= j; k++) new_path.push_back(path[k]);
                new_path.insert(new_path.end(), insert_path.begin(), insert_path.end());
                new_path.push_back(path[turning_index[i]-1]);
                // i为终点的时候才将此第三点加入新路径，否则第三点会在下一段加入
                if(i == turning_index.size()-1)new_path.push_back(path[turning_index[i]]);
                find_path = true;
                break;
            }
        }

        if(find_path) // 找到对这3个点的优化，可以直接跳过两个点了，因为消失了一个点
        {
            i++;
            if(i == turning_index.size()-1)
            {
                for(int j = turning_index[i-1]; j <= turning_index[i]; j++)
                    new_path.push_back(path[j]);
            }
        }
        else // 新路径等于将原路径这一段
        {
            for(int j = turning_index[i-2]; j < turning_index[i-1]; j++)
                new_path.push_back(path[j]);
            if(i == turning_index.size()-1)
            {
                for(int j = turning_index[i-1]; j <= turning_index[i]; j++)
                    new_path.push_back(path[j]);
            }
        }
    }
    if(turning_index.size() > 2)
    {
    	path.clear();
    	path.insert(path.begin(),new_path.begin(),new_path.end());
	}
    
}


void far_barrier(vector<Point>& path, bool is_load)
{
    int upleft = 0, upright = 0, downleft = 0, downright = 0;
    int a = 0, b = 0, c = 0, d = 0;
    int idx, idy, far_idx, far_idy;
    for (int t = 1; t < (int)path.size() - 1; t++) // 注意起始终点都不作处理
    {
        // 路径居中，向左右侧障碍物远离

        idx = path[t].index_x;
        idy = path[t].index_y;
        in_danger(idx, idy, is_load, path[path.size() - 1], upleft, upright, downleft, downright);
        if (upleft + upright + downleft + downright == 0) continue;
        if (upleft + upright + downleft + downright == 1) {
            if (upleft) {
                path[t].x -= 0.25;
                path[t].y -= 0.25;
            }
            else if (upright) {
                path[t].x -= 0.25;
                path[t].y += 0.25;
            }
            else if (downleft) {
                path[t].x += 0.25;
                path[t].y -= 0.25;
            }
            else if (downright) {
                path[t].x += 0.25;
                path[t].y += 0.25;
            }
        }
        else {
            if (is_alarm(idx + 1, idy)) {
                path[t].y += 0.25;
            }
            if (is_alarm(idx - 1, idy)) {
                path[t].y -= 0.25;
            }
            if (is_alarm(idx, idy - 1)) {
                path[t].x += 0.25;
            }
            if (is_alarm(idx, idy + 1)) {
                path[t].x -= 0.25;
            }
        }
    }
}

