#include "main.h"
#include "workspace.h"
#include "a_star.h"
#include "robot.h"

#include <iostream>

using namespace std;

vector<Workspace *> Workspace::workspaces;
vector<Workspace *> Workspace::enemy_workspaces;
vector<Workspace*> Workspace::workspaces_type_mp[10];
vector<Workspace*> Workspace::enemy_workspaces_type_mp[10];
// 考虑4 5 6号工作台是否缺另一半原料
bool Workspace::is_first()
{
    int count = 0;
    if (type < 7)
    { // 456的
        for (int i = 1; i <= 3; i++)
        {
            if (reserved[i] == 1 || resource_state[i] == 1)
            {
                // 因为该函数在第二层循环里使用，故已经被选为卖点，不用判断已满的情况
                count++;
            }
        }
        if (count % 2 == 0)
        { // 偶数
            return false;
        }
        else
        {
            return true;
        }
    }
    else if (type == 7 && mapID == 1)
    { // 卖给7
        for (int i = 4; i <= 6; i++)
        {
            if (reserved[i] == 1 || resource_state[i] == 1)
            {
                // 因为该函数在第二层循环里使用，故已经被选为卖点，不用判断已满的情况
                count++;
            }
        }
        if (count == 2 || count == 1)
            return true;
        else
            return false;
    }
    return false;
    for (int i = 1; i <= 7; i++)
    {
        if (reserved[i] == 1 || resource_state[i] == 1)
        {
            // 因为该函数在第二层循环里使用，故已经被选为卖点，不用判断已满的情况
            count++;
        }
    }
    if (count % 2 == 0)
    { // 偶数
        return false;
    }
    else
    {
        return true;
    }
}

// 工作台类型为456，由原料格状态和remain_time更新影响因子
double Workspace::impact_factor[3] = {1.0, 1.0, 1.0};
void Workspace::init_impact_factor()
{
    for (int i = 0; i < 3; i++)
    {
        impact_factor[i] = 1.0;
    }
}
// 卖点为7时，根据原料格状态，调整456（为买点、卖点）的优先级，
void Workspace::update_impact_factor()
{
    if (type != 7)
    {
        return;
    }
    if (is_update == 1)
    {
        return;
    }
    for (int i = 0; i < 3; i++)
    {
        if (resource_state[i + 4] == 0 && reserved[i + 4] == 0)
        {
            impact_factor[i] -= 0.13; // 可调参数
        }
    }
    is_update = 1;
}

int Workspace::visit_times[3] = {0, 0, 0};

/**
 * @brief 构造函数
 *
 * @param type
 * @param x
 * @param y
 */
Workspace::Workspace(int type, double x, double y, int index_x, int index_y, char color)
    : type(type), x(x), y(y), index_x(index_x), index_y(index_y), color(color)
{
    for (int i = 1; i <= 7; i++)
    {
        resource_state[i] = 0;
        reserved[i] = 0;
        is_update = 0;
    }
    remain_preduct_time = 0;
    preduct_state = 0;
    is_far_center = 0; // 初始化为非远离worspace集群中心的点
    // count_of_be_attacking = 0;
    is_safe = 1;
    unsafe_frame = -1; // 初始化为-1,代表最近无检测到附近有敌方机器人
    can_buy_to_attack = 1;

    attack_failed_times = 0;
}

void Workspace::init_next_workspace()
{
    for (auto wk : Workspace::workspaces)
    {
        if (workspace_sell_ok[type][wk->type] && // 可卖
            allpaths_valid[index_x][index_y][wk->id][1])
        { // 且带货可达
            next_workspace.push_back(wk->id);
            // cerr<<"id = "<<id<<" type = "<<type<<" next->id="<<wk->id<<" next->type"<<wk->type<<endl;
        }
    }
}

void Workspace::init_frame(vector<string> vec)
{
    type = atoi(vec[0].c_str());
    x = stod(vec[1]);
    y = stod(vec[2]);
    remain_preduct_time = atoi(vec[3].c_str()); // -1表示没有生产 0表示产品格满而阻塞 ，正常表示生产剩余帧数
    resource_state_o = atoi(vec[4].c_str());    // 原材料格状态
    preduct_state = atoi(vec[5].c_str());       // 产品格状态

    for (int i = 1; i <= 7; i++)
        resource_state[i] = (((resource_state_o >> i) & 1) == 1); // 原材料格状态
    is_update = 0;                                                // 记录状态 ：没有更新影响因子
    
    int attack_cnt = 0;
    for(Robot* rot : Robot::enemy_robots){
        if(abs(rot->index_x - index_x) + abs(rot->index_y - index_y) <= 2){//near
            attack_cnt++;
            if(is_safe && unsafe_frame == -1) // 第一次记录到有敌人在此工作台，记录下看到了帧数
                unsafe_frame = frameID; 
            else if((attack_cnt >= 2 || (attack_cnt && color=='r') || (in_corner() >= 3 && attack_cnt) || 
                    (in_corner() >= 2 && attack_cnt && color=='b')) &&
                     is_safe && (unsafe_frame + 50 <= frameID)){
                // 距离第一次看见敌人在此工作台时间超过50帧，设置此工作台被占据
                unsafe_frame = frameID; 
                is_safe = 0;
                if(type == 4){
                    num_of_4 --;
                }
                if(type == 5){
                    num_of_5 --;
                }
                if(type == 6){
                    num_of_6 --;
                }
                if(type == 7){
                    num_of_seven --;
                }
                break;
            }
            else if(is_safe == 0) 
            {
                // 此工作台已经为危险工作台，持续查看其在此处，更新最近一次看到的时间
                unsafe_frame = frameID;
            }
        }
        else // 该工作台附近没有敌人
        {
            if(is_safe == 0 && (unsafe_frame + 100 <= frameID) && (attack_failed_times < 2)) // 危险工作台已经超过50帧没看见敌人了，设置回安全
            {
                is_safe = 1;
                unsafe_frame = -1;
                if(type == 4){
                    num_of_4 ++;
                }
                if(type == 5){
                    num_of_5 ++;
                }
                if(type == 6){
                    num_of_6 ++;
                }
                if(type == 7){
                    num_of_seven ++;
                }
            }
        }
    }

    if(attack_cnt == 0 && Robot::enemy_robots.size() == 4) 
    {
        // 对于看到4个敌方机器人都没占据，判断是否满足100帧解除被占据状态
        if(is_safe == 0 && unsafe_frame + 100 <= frameID) // 危险工作台已经超过50帧没看见敌人了，设置回安全
        {
            is_safe = 1;
            unsafe_frame = -1;
            if(type == 4){
                num_of_4 ++;
            }
            if(type == 5){
                num_of_5 ++;
            }
            if(type == 6){
                num_of_6 ++;
            }
            if(type == 7){
                num_of_seven ++;
            }
        }
    }    
}


bool Workspace::is_resouce_no_reserved(){
    for(int i = 1 ; i < 7; i++) {
        // 可接受产品i 且 该位不存在原料 ，则不为满
        if(workspace_sell_ok[i][type] && reserved[i] ) return false;
    }
    return true;
}
bool Workspace::is_full_resource() {
    for(int i = 1 ; i < 7; i++) {
        // 可接受产品i 且 该位不存在原料 ，则不为满
        if(workspace_sell_ok[i][type] && !resource_state[i] ) return false;
    }
    return true;
}
bool Workspace::is_full_reserved_and_resource(){
    for(int i = 1 ; i < 7; i++) {
        // 可接受产品i 且 该位不存在原料 , 且没有被预定，则不为满
        if(workspace_sell_ok[i][type] && !resource_state[i] && !reserved[i]) return false;
    }
    return true;
}


bool Workspace::is_near_barrier()
{
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (graph[index_x + i][index_y + j] == '#')
            {
                return true;
            }
        }
    }
    return false;
}

bool Workspace::is_corner_workspace()
{
    // 工作台位于地图四角
    if ((index_x == 0 && index_y == 0) || (index_x == 0 && index_y == 99) ||
        (index_x == 99 && index_y == 0) || (index_x == 99 && index_y == 99))
    {
        return true;
    }

    // 工作台上下左右四个方向是否有障碍物
    int left = 0, right = 0, up = 0, down = 0;
    if (graph[index_x - 1][index_y] == '#')
    {
        up = 1;
    }
    if (graph[index_x + 1][index_y] == '#')
    {
        down = 1;
    }
    if (graph[index_x][index_y - 1] == '#')
    {
        left = 1;
    }
    if (graph[index_x][index_y + 1] == '#')
    {
        right = 1;
    }

    // 判断工作台是否在障碍物边角
    if ((left + right) && (up + down))
    {
        return true;
    }

    return false;
}

int Workspace::in_corner(){
    /********************用于选点*****************/
    // 0 1表示空地
    //2表示努努力能撞开
    //3表示基本卡死
    //4表示完全卡死，应该不会有4

    //考虑一面贴墙的情况？
    if(is_near_barrier()){
        if(is_corner_workspace())
            return 3;
    }

    int left_barrier = 0,right_barrier = 0,up_barrier = 0,down_barrier = 0;
    int step = 2;
    if(graph[index_x + 2][index_y] == '#'){
        right_barrier = 1;
    }
    if(graph[index_x + 1][index_y] == '#'){
        right_barrier = 1;
    }
    if(graph[index_x - 2][index_y] == '#'){
        left_barrier = 1;
    }
    if(graph[index_x - 1][index_y] == '#'){
        left_barrier = 1;
    }
    if(graph[index_x][index_y + 2] == '#'){
        up_barrier = 1;
    }
    if(graph[index_x][index_y + 1] == '#'){
        up_barrier = 1;
    }
    if(graph[index_x][index_y - 2] == '#'){
        down_barrier = 1;
    }
    if(graph[index_x][index_y - 1] == '#'){
        down_barrier = 1;
    }

    if((right_barrier || left_barrier) && (up_barrier || down_barrier)){
        return right_barrier + left_barrier + up_barrier + down_barrier;
    }
    return 0;//表示在空地
}

int Workspace:: in_danger_corner(){
    int danger_dx[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
    int danger_dy[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };

    bool danger = false;

    int around[8] = { 0 };
    int upleft, upright, downleft, downright;
    for (int i = 0; i < 8; i++) {
        around[i] = in_danger(index_x + danger_dx[i], index_y + danger_dy[i], 1, 
           Point(-1, -1), upleft, upright, downleft, downright) ? 1 : 0;
    }
    // 1代表有障碍物
    int ul = 0, ur = 0, dl = 0, dr = 0;
    ul = around[0] || around[1] || around[3] ? 1 : 0;
    ur = around[1] || around[2] || around[4] ? 1 : 0;
    dl = around[3] || around[5] || around[6] ? 1 : 0;
    dr = around[4] || around[7] || around[6] ? 1 : 0;

    // 为0, 空旷的
    // 为1, 只有一个边角处于危险区
    // 为2, 可能是一面有墙(危险区), 也可能是对角有墙(危险区), 占据或者是被占据都有困难
    // 3以上 墙角 

    in_danger_corner_score = ul + ur + dl + dr;

    if(in_danger(index_x, index_y, 1, Point(-1, -1), upleft, upright, downleft, downright)){
        in_danger_corner_score = 4;
    }
    return in_danger_corner_score;
}


int Workspace::num_of_ready_resouce_of_seven(){
    if (type != 7)
    {
        return 0;
    }
    int count = 0;
    for (int i = 4; i <= 6; i++)
    {
        if(reserved[i] != 0 || resource_state[i] != 0){
            count++;
        }
    }
    return count;
}