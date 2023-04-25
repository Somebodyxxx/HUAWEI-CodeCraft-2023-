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

/* a_star中邻居的个方位位置以及距离, 上右下左 左上 右上 右下 左下 */
const int index_dx[8] = {0, 1,  0, -1, -1, 1,  1, -1};
const int index_dy[8] = {1, 0, -1,  0,  1, 1, -1, -1};
const double diagonal = 0.5 * sqrt(2);
const double dis[8] = {0.5, 0.5, 0.5, 0.5,
                       diagonal, diagonal, diagonal, diagonal};

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
double F[width][width];                // 每个点路径通过的热力值


/**
 * @brief 增加某点的热力值，当前点增加f=1，附近点增加neig_f=0.25
 * 
 * @param index_x 
 * @param index_y 
 */
void F_add(int index_x, int index_y)
{
    int f = 0.01;
    double neig_f = 0.00125;
    F[index_x][index_y] += f;
    for(int i = 0; i < 8; i++){
        int dx = index_x+index_dx[i];
        int dy = index_y+index_dy[i];
        if(in_range(dx, dy))
            F[dx][dy] += neig_f;
    }
}


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
bool in_danger(int index_x, int index_y, bool is_load, const Point des, int& upleft, int& upright, int& downleft, int& downright)
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
            if (is_alarm(index_x, index_y + 1) && !is_alarm(index_x, index_y - 1) && is_alarm(index_x, index_y - 2) ||
                is_alarm(index_x, index_y - 1) && !is_alarm(index_x, index_y + 1) && is_alarm(index_x, index_y + 2) ||
                is_alarm(index_x + 1, index_y) && !is_alarm(index_x - 1, index_y) && is_alarm(index_x - 2, index_y) ||
                is_alarm(index_x - 1, index_y) && !is_alarm(index_x + 1, index_y) && is_alarm(index_x + 2, index_y)) {
                danger = false;
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
    if(Workspace::workspaces[workspace_id] == NULL) return false;
    // 构建终点
    Point DES(99 - floor(Workspace::workspaces[workspace_id]->y / 0.5), floor(Workspace::workspaces[workspace_id]->x / 0.5));

    return a_star(SOUR, DES, is_load, path, path_dis);
}

bool a_star(Point SOUR, Point DES, bool is_load, vector<Point> &path, double &path_dis)
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

        /* 将该点周围8个邻居加入open集合 */
        for (int i = 0; i < 8; i++)
        {
            // 邻居的栅格索引
            int new_index_x = now_p.index_x + index_dx[i];
            int new_index_y = now_p.index_y + index_dy[i];
            
            // 判断障碍物危险
            if (in_danger(new_index_x, new_index_y, is_load, DES, up, down, left, right))
                continue; 

            // 斜向运动时,不可以转交处斜向
            if(i == 4 && (up_center || left_center)) continue;
            if(i == 5 && (up_center || right_center)) continue;
            if(i == 6 && (down_center || right_center)) continue;
            if(i == 7 && (down_center || left_center)) continue;

            // 计算当前节点的cost，（当前点到源点的dis + open点到当前点的dis） + h(x)open点欧氏距离到目的点
            double new_dist = distance + dis[i];
            // 距离障碍物影响参数，非载货的距离障碍物可以近一些
            double D_C = 3.0/(D[new_index_x][new_index_y]); 
            double new_cost = new_dist + sqrt(pow(0.5*(new_index_x - DES.index_x), 2) + pow(0.5*(new_index_y - DES.index_y), 2))
                                + D_C; // + F[new_index_x][new_index_y];

            // 计算到当前点i方向拓展所需要的转弯转折次数
            int turn_cnt = 0;
            if(direct[now_p.index_x][now_p.index_y] != i) 
                turn_cnt = turn[now_p.index_x][now_p.index_y] + 1;
            else 
                turn_cnt = turn[now_p.index_x][now_p.index_y];

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

    // 对路径处理远离障碍物
    far_barrier(path, is_load);

    // // 处理角落区域工作台买路径最后一个点到DES方向必须是上下左右四个方向
    // if(in_danger(DES.index_x, DES.index_y, 1, DES, up, down, left, right) &&
    //     direct[DES.index_x][DES.index_y] > 3)
    // {
    //     int ix = DES.index_x;
    //     int iy = DES.index_y;
    //     if(direct[DES.index_x][DES.index_y] == 4) iy-=2; // 左上到上
    //     else if(direct[DES.index_x][DES.index_y] == 5) iy-=2; // 右上到上
    //     else if(direct[DES.index_x][DES.index_y] == 6) iy+=2; // 右下到下
    //     else if(direct[DES.index_x][DES.index_y] == 7) iy+=2; // 左下到下
    //     path.insert(path.end()-1, Point(ix, iy));
    // }

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
 * @brief 对某条路径执行远离障碍物操作
 * 
 * @param path 路径数组
 * @param is_load 
 */
//void far_barrier(vector<Point>& path, bool is_load)
//{   
//    int upleft = 0, upright = 0, downleft = 0, downright = 0;
//    int a = 0, b = 0, c = 0, d = 0;
//    int idx, idy, far_idx, far_idy;
//    for (int t = 1; t < (int)path.size() - 1; t++) // 注意起始终点都不作处理
//    {
//        // 路径居中，向左右侧障碍物远离
//       
//        idx = path[t].index_x;
//        idy = path[t].index_y;
//        in_danger(idx, idy, is_load, path[path.size() - 1], upleft, upright, downleft, downright);
//        if (upleft + upright + downleft + downright == 0) continue;
//        if (upleft + upright + downleft + downright == 1) {
//            if (upleft) {
//                if (!in_danger(idx + 1, idy - 1, is_load, path[path.size() - 1], a, b, c, d)) {
//                    path[t] = Point(idx + 1, idy - 1);
//                }
//                else {
//                    path[t].x -= 0.25;
//                    path[t].y -= 0.25;
//                }
//            }
//            else if (upright) {
//                if (!in_danger(idx - 1, idy - 1, is_load, path[path.size() - 1], a, b, c, d)) {
//                    path[t] = Point(idx - 1, idy - 1);
//                }
//                else {
//                    path[t].x -= 0.25;
//                    path[t].y += 0.25;
//                }
//            }
//            else if (downleft) {
//                if (!in_danger(idx + 1, idy + 1, is_load, path[path.size() - 1], a, b, c, d)) {
//                    path[t] = Point(idx + 1, idy + 1);
//                }
//                else {
//                    path[t].x += 0.25;
//                    path[t].y -= 0.25;
//                }
//            }
//            else if (downright) {
//                if (!in_danger(idx - 1, idy + 1, is_load, path[path.size() - 1], a, b, c, d)) {
//                    path[t] = Point(idx - 1, idy + 1);
//                }
//                else {
//                    path[t].x += 0.25;
//                    path[t].y += 0.25;
//                }
//            }
//        }
//        else {
//            if (is_alarm(idx + 1, idy)) {
//                path[t].y += 0.25;
//            }
//            if (is_alarm(idx - 1, idy)) {
//                path[t].y -= 0.25;
//            }
//            if (is_alarm(idx, idy - 1)) {
//                path[t].x += 0.25;
//            }
//            if (is_alarm(idx, idy + 1)) {
//                path[t].x -= 0.25;
//            }
//        }
//    }
//}

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


/**
 * @brief 初始化图上所有点到所有workspace的路径，障碍物除外
 *          处理了远离障碍物移动点。
 *
 */
void init_allpaths()
{
    // 初始化
    memset(allpaths_dis, 0.0, sizeof allpaths_dis);
    memset(allpaths_valid, true, sizeof allpaths_valid); // 初始化为
    memset(F, 0.0, sizeof F);

    // 初始化距离障碍物
    init_D();

    // 通过4个机器人判断可达与不可达工作台目标
    // 每个机器人a_star搜索所有的workspace，给机器人添加可达的工作台
    // 则每个机器人可达与不可达的workspace之间可达关系也被划分出来
    vector<int> reachable_set, unreachale_set;
    for(int i = 0; i < 4; i++) {
        Robot* rob = Robot::robots[i];
        int index_x, index_y;
        get_index_by_coordinate(rob->x, rob->y, index_x, index_y);
        Point rob_point(index_x, index_y); 

        reachable_set.clear();
        unreachale_set.clear();

        // 遍历每一个workspace,设置该机器人起始位置到这些工作台是否可达
        // 设置机器人是否对这些 工作台是否可达
        // 设置机器人的alllpaths路径
        for(int j = 0; j < nums_of_workspaces; j++){ 
            bool valid = a_star(rob_point, j, 0, allpaths[index_x][index_y][j][0], allpaths_dis[index_x][index_y][j][0]);
            allpaths_valid[index_x][index_y][j][0] = valid;
            rob->workspace_is_reachable[j] = valid; // 该机器人到此是否可达
            // 如果可达，把带货路径也求了，不带货都走不到，带货肯定不可通
            // 处理路径点集
            if(valid){
                reachable_set.push_back(j);
                allpaths_valid[index_x][index_y][j][1] =
                    a_star(rob_point, j, 1, allpaths[index_x][index_y][j][1], allpaths_dis[index_x][index_y][j][1]);
            }
            else {
                unreachale_set.push_back(j);
            }
        }

        // workspace之间归类可达不可达
        for(auto w1 : reachable_set) {
            index_x = Workspace::workspaces[w1]->index_x;
            index_y = Workspace::workspaces[w1]->index_y;
            // 集合之间可达不用赋值，初始化全部可达
            // 相同集合之间不可达
            for(auto w2 : unreachale_set) {
                allpaths_valid[index_x][index_y][w2][0] = 0;
                allpaths_valid[index_x][index_y][w2][1] = 0;
                // 相反 w2到w1也不可达
                allpaths_valid[Workspace::workspaces[w2]->index_x][Workspace::workspaces[w2]->index_y][w1][0] = 0;
                allpaths_valid[Workspace::workspaces[w2]->index_x][Workspace::workspaces[w2]->index_y][w1][1] = 0;
            }
        }
    }

    // 构建机器人和workspace的点集
    Point sours[nums_of_workspaces];
    for (int k = 0; k < nums_of_workspaces; k++)
    {
        get_index_by_coordinate(Workspace::workspaces[k]->x, Workspace::workspaces[k]->y, sours[k].index_x, sours[k].index_y);
    }

    // 初始化工作台到工作台之间的路径和机器人起始点到工作台之间的路径
    for (int n = 0; n < nums_of_workspaces; n++)
    {
        Point sour = sours[n];
        int i = sour.index_x;
        int j = sour.index_y;
        // 注意K=n每个循环只处理到没有处理(n,k)对，处理一对一次可以得到的路径直接赋值给双向的路径
        for (int k = n; k < nums_of_workspaces; k++) 
        {
            if(!allpaths_valid[i][j][k][0]) continue; // 已知不可达就跳过

            for (int l = 0; l < 2; l++)
            {
                // 寻路
                bool valid = a_star(sour, k, l, allpaths[i][j][k][l], allpaths_dis[i][j][k][l]);
                allpaths_valid[i][j][k][l] = valid;
            
                // 对调起点工作台和终点工作台，直接赋值路径
                // 且对调后的终点是indanger的
                int up = 0, down = 0, left = 0, right = 0;
                if(in_danger(i, j, l, sour, up, down, left, right))
                { // 对调后indanger 就标记路径危险的
                    allpaths_valid[sours[k].index_x][sours[k].index_y][n][l] = false;
                }
                else 
                { 
                    vector<Point> temp_vec(allpaths[i][j][k][l]);
                    reverse(temp_vec.begin(), temp_vec.end());
                    allpaths[sours[k].index_x][sours[k].index_y][n][l] = temp_vec;
                    allpaths_dis[sours[k].index_x][sours[k].index_y][n][l] = allpaths_dis[i][j][k][l];
                    allpaths_valid[sours[k].index_x][sours[k].index_y][n][l] = allpaths_valid[i][j][k][l];
                }
            }
        }
    }
}



/*****************************TEST*************************************/

// int main() {
//     // mook graph
//     nums_of_workspaces = 0;
//     for(int i = 0; i < width; i++)
//         for(int j = 0; j < width; j++){
//             cin>>graph[i][j];
//             // mook worksapces
//             if (graph[i][j] <= '9' && graph[i][j] >= '1')
//             { // 工作台初始化点
//                 (Workspace::workspaces).push_back(new Workspace((graph[i][j]-'0'), j * 0.5 + 0.25, 50 - i * 0.5 - 0.25, i, j));
//                 nums_of_workspaces++;
//             }
//             if (graph[i][j] == 'A')
//             { // 机器人初始化点
//                 (Robot::robots).push_back(new Robot(j * 0.5 + 0.25, 50 - i * 0.5 - 0.25));
//             }
//         }

//     // 计算时间测试
//     clock_t start, end;
//     start = clock();

//     int test_workspace_id = 12;

//     // Point sour(24, 39); // 第一个机器人位置
//     // vector<Point> path;

//     // // 单路径寻路测试，将路径用X标注在地图，使用map4
//     // double path_dis;
//     // cout<<a_star(sour, test_workspace_id, 0, path, path_dis)<<endl;;
//     // for(auto item: path){
//     //     cout<<item.index_x<<","<<item.index_y<<";  "<<item.x<<","<<item.y<<endl;
//     //     graph[item.index_x][item.index_y] = 'X';
//     // }
//     // for(int i = 0; i < width; i++) {
//     //     for(int j = 0; j < width; j++){
//     //         cout<<graph[i][j];
//     //     }
//     //     cout<<endl;
//     // }


//     // 测试robot0起始位置到某个工作台的路径搜索结果，测试allpaths结果
//     init_allpaths();

//     int index_x,index_y; 
//     get_index_by_coordinate(Robot::robots[2]->x, Robot::robots[2]->y, index_x, index_y);
//     cout<<index_x<<" "<<index_y<<endl;
//     for(auto item: allpaths[index_x][index_y][test_workspace_id][0]){
//         cout<<item.index_x<<","<<item.index_y<<";  "<<item.x<<","<<item.y<<endl;
//         graph[item.index_x][item.index_y] = 'X';
//     }
//     if(allpaths[index_x][index_y][test_workspace_id][0].size() == 0) cout<<0<<endl;
//     else cout<<1<<endl;

//     for(int i = 0; i < width; i++) {
//         for(int j = 0; j < width; j++){
//             cout<<graph[i][j];
//         }
//         cout<<endl;
//     }
//         cout<<endl;
//     // for(int i = 0; i < width; i++) {
//     //     for(int j = 0; j < width; j++){
//     //         cout<<F[i][j]<<" ";
//     //     }
//     //     cout<<endl;
//     // }

//     // 结束时间
//     end = clock();
//     double elapsedTime = static_cast<double>(end-start) / CLOCKS_PER_SEC ;
//     //clock()以毫秒的形式展现，因此需要除以 CLOCKS_PER_SEC 来实现转换
//     //static_cast<double>的作用是将结果转换为double类型
//     printf("CPU PROCESSING TIME: %f",elapsedTime);
//     return 0;
// }