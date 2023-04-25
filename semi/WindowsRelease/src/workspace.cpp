#include "main.h"
#include "workspace.h"
#include "a_star.h"

#include <iostream>

using namespace std;

//考虑4 5 6号工作台是否缺另一半原料
bool Workspace::is_first(){
    int count = 0;
    if(mapID == 1 || mapID == 2){
        if(type < 7) { // 456的
            for(int i = 1;i<=3;i++){
                if(reserved[i] == 1 || resource_state[i] == 1){
                    //因为该函数在第二层循环里使用，故已经被选为卖点，不用判断已满的情况
                    count ++;
                }
            }
            if(count % 2 == 0){//偶数
                return false;
            }else{
                return true;
            }
        }
        else if(type == 7 && mapID == 1){ // 卖给7
            for(int i = 4;i<=6;i++){
                if(reserved[i] == 1 || resource_state[i] == 1){
                    //因为该函数在第二层循环里使用，故已经被选为卖点，不用判断已满的情况
                    count ++;
                }
            }
            if(count == 2 || count == 1) return true;
            else return false;
        }
        return false;
    }
    for(int i = 1;i<=7;i++){
        if(reserved[i] == 1 || resource_state[i] == 1){
            //因为该函数在第二层循环里使用，故已经被选为卖点，不用判断已满的情况
            count ++;
        }
    }
    if(count % 2 == 0){//偶数
        return false;
    }else{
        return true;
    }
}

//工作台类型为456，由原料格状态和remain_time更新影响因子
double Workspace::impact_factor[3] = {1.0,1.0,1.0};
void Workspace::init_impact_factor(){
    for (int i = 0; i < 3; i++)
    {
        impact_factor[i] = 1.0;
    }
}
//卖点为7时，根据原料格状态，调整456（为买点、卖点）的优先级，
void Workspace::update_impact_factor(){
    if(type != 7){
        return;
    }
    if(is_update == 1){
        return;
    }
    for (int i = 0; i < 3; i++)
    {
        if(resource_state[i+4] == 0 && reserved[i+4] == 0){
            impact_factor[i] -= 0.13;//可调参数
        }
    }
    is_update = 1;
}

int Workspace::visit_times[3] = { 0,0,0 };

vector<Workspace*> Workspace::workspaces;

/**
 * @brief 构造函数
 * 
 * @param type 
 * @param x 
 * @param y 
 */
Workspace::Workspace(int type, double x,double y, int index_x, int index_y)
    :type(type),x(x),y(y),index_x(index_x),index_y(index_y){
    for(int i = 1; i <= 7; i++){
        resource_state[i] = 0;
        reserved[i] = 0;
        is_update = 0;
    }
    remain_preduct_time = 0;
    preduct_state = 0;

}

void Workspace::init_next_workspace() {
    for(auto wk : Workspace::workspaces) {
        if(workspace_sell_ok[type][wk->type] &&           // 可卖
            allpaths_valid[index_x][index_y][wk->id][1]){ // 且带货可达
            next_workspace.push_back(wk->id);
            // cerr<<"id = "<<id<<" type = "<<type<<" next->id="<<wk->id<<" next->type"<<wk->type<<endl;
        }
    }
}

void Workspace::init_frame(vector<string> vec) {
    type = atoi(vec[0].c_str());
    x = stod(vec[1]);
    y = stod(vec[2]);
    remain_preduct_time = atoi(vec[3].c_str()); // -1表示没有生产 0表示产品格满而阻塞 ，正常表示生产剩余帧数
    resource_state_o = atoi(vec[4].c_str()); //原材料格状态
    preduct_state = atoi(vec[5].c_str()); //产品格状态

    for(int i = 1; i <= 7; i++) resource_state[i] = (((resource_state_o >> i) & 1) == 1); //原材料格状态
    is_update = 0;//记录状态 ：没有更新影响因子
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

bool Workspace::is_near_barrier(){//TODO:障碍物旁边的工作台降低权值
    for(int i = -1;i<= 1 ;i++){
        for(int j = -1;j <= 1; j ++){
            if(graph[index_x + i][index_y + j] == '#'){
                return true;
            }
        }
    }
    return false;
}

bool Workspace::is_corner_workspace(){//TODO:边角的工作台降低权值
    //工作台位于地图四角
    if((index_x == 0 && index_y == 0) || (index_x == 0 && index_y == 99) ||
     (index_x == 99 && index_y == 0) || (index_x == 99 && index_y == 99)){
        return true;
    }

    // 工作台上下左右四个方向是否有障碍物
    int left = 0,right = 0,up = 0,down = 0;
    if(graph[index_x - 1][index_y] == '#'){
        up = 1;
    }
    if(graph[index_x + 1][index_y] == '#'){
        down = 1;
    }
    if(graph[index_x][index_y - 1] == '#'){
        left = 1;
    }
    if(graph[index_x][index_y + 1] == '#'){
        right = 1;
    }

    //判断工作台是否在障碍物边角
    if((left + right) && (up + down)){
        return true;
    }

    return false;
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