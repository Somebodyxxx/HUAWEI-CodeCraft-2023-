#include "robot.h"
#include "workspace.h"
#include "main.h"
#include "a_star.h"

#include <math.h>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <map>
#include <stdio.h>

using namespace std;
#define PI 3.14159265358979323846


#define in_range(i,j) (i >= 0 && i < width && j >= 0 && j < width)

int Robot::money = 200000;

vector<Robot *> Robot::robots;

/**
 * @brief 判断是否有路径点冲突：点相同地方 且 距离源点有冲突
 * 
 * @param mp 
 * @param current_path 
 * @param dis_vec 
 * @return true 
 * @return false 
 */


void Robot::init_frame(vector<string> vec)
{
    is_near_workspace = stoi(vec[0]);

    load_id = atoi(vec[1].c_str());
    if (load_id == 0)
    {
        r = 0.45;
    }
    else
    {
        r = 0.53;
    }

    holdtime = atoi(vec[2].c_str());
    collidetimes = atoi(vec[3].c_str());
    w = stod(vec[4]);
    v[0] = stod(vec[5]);
    v[1] = stod(vec[6]);
    rotation = stod(vec[7]);
    x = stod(vec[8]);
    y = stod(vec[9]);

    get_index_by_coordinate(x,y,index_x,index_y);
    update_all_robots_next_path();
    BFS_fail = 0;
}
void Robot::simple_seek(){//TODO:没7关策略
//TODO: 如果合成7代价太高，（和9对比），手动关手
    if(!task.empty()){
        return;
    }
    //接任务
    double cost_sum_dist = 100000.0;
    double current_sum_dist;
    double nine_cost = 100000.0;
    vector<Point> path_buy;
    vector<Point> path_sell;
    vector<Point> path_sell_nine;
    int target1 = -1;
    int target2 = -1;
    int target2_nine = -1;
    vector<Point> temp_buy;
    double dist_buy = 100000.0;
    double dist_sell = 100000.0;
    double dist_sell_nine = 100000.0;
    double time_buy,time_sell,time_sell_nine;
    double record_time_buy,record_time_sell;
    double record_time_sao = 100000.0;

    //临时变量，机器人当前点的索引
    // int temp_index_x, temp_index_y;
    // get_index_by_coordinate(x,y,temp_index_x,temp_index_y);

    for(Workspace* wk : Workspace::workspaces){//买点
        // 不可达点跳过
        if(!workspace_is_reachable[wk->id]){
            // cerr<<wk->id << "unreachable"<<endl;
            continue;
        } 
        if(wk->reserved[wk->type] == 1){//被预定
            // cerr<<"has reserved..skip."<<endl;
            continue;
        }
        temp_buy =  allpaths[index_x][index_y][wk->id][0];
        // for(Point ele: allpaths[temp_index_x][temp_index_y][wk->id][0]){
        //     cerr<<ele.index_x<<" "<<ele.index_y<<endl;
        // }
        // cerr<<wk->id <<"buy_path_size "<<temp_buy.size() <<endl;
        if(temp_buy.empty()){
            if(a_star(Point(index_x,index_y),wk->id,0,temp_buy,dist_buy)){
                allpaths[index_x][index_y][wk->id][0] = temp_buy;
                allpaths_dis[index_x][index_y][wk->id][0] = dist_buy;
            }else{
                //cerr<<wk->id <<"has no road ,skiped... " <<endl;
                continue;
            }
        }
        dist_buy = allpaths_dis[index_x][index_y][wk->id][0];//dist赋值

        time_buy = get_time_by_dist(dist_buy);
        if (wk->preduct_state == 1 || ( (time_buy + reach_extern_time > wk->remain_preduct_time) && (wk->remain_preduct_time != -1) )){//没生产好
            //没问题
        }else{
            continue;
        }

        for (int target2Id : wk->next_workspace){
            if(allpaths_valid[wk->index_x][wk->index_y][target2Id][1]){
                //可达
            }else{
                continue;
            }
            if(Workspace::workspaces[target2Id]->reserved[wk->type] == 1)//该原料格被预定
            {
                continue;
            }
            dist_sell = allpaths_dis[wk->index_x][wk->index_y][target2Id][1];//dist赋值，工作点之间，必定有值
            

            //优化，使得卖给4 5 6平均。
            if(num_of_seven && num_of_4 && num_of_5 && num_of_6 && frameID < close_do7_frame){
                // cerr<< "hello 1.."<<endl;
                // 获得456类型台中最大和最小访问次数
                int max_visit = *max_element(Workspace::visit_times, Workspace::visit_times + 3);
                int min_visit = *min_element(Workspace::visit_times, Workspace::visit_times + 3);
                // 当前是最大访问次数工作台且比最小访问次数大于2了，且当前全空，就直接跳过，不考虑给他送了
                if (Workspace::visit_times[Workspace::workspaces[target2Id]->type - 4] == max_visit
                    && ((max_visit - min_visit) > 2)) {
                        // if(Workspace::workspaces[target2Id]->resource_state_o == 0){
                        //     continue;
                        // }
                        continue;
                }
            }

            time_sell = get_time_by_dist(dist_sell);
            // 原料格空或 原料格满且生产时间小于到达时间
            if ( (Workspace::workspaces[target2Id]->resource_state[wk->type] == 0) || //预定前面判断了
                (Workspace::workspaces[target2Id]->is_full_resource() &&              // 原料格满
                    Workspace::workspaces[target2Id]->remain_preduct_time != -1 &&       // 正在生产
                    (Workspace::workspaces[target2Id]->remain_preduct_time  < (time_buy + time_sell + reach_extern_time * 2.)) && // 到达的时候生产完成
                    Workspace::workspaces[target2Id]->preduct_state == 0) )            // 产品格必须为空（否则可能一直阻塞生产不出来）
            {
                //放宽卖点被选择条件后，该卖点没问题
            }else{
                continue;
            }

            // 增加判断游戏结束时候能不能卖出去，否则跳过
            if ((double)frameID + time_buy + time_sell + end_extern_time > 15000.0){
                continue;
            }
            
            // int cnt = 0;
            // for (auto i : Workspace::workspaces[target2Id]->reserved) cnt += i.second;
            // if (cnt > 2) continue;

            //// 计算和比较花销选择最小的情况
            //double time_c = func(time_sell); // 时间系数
            //double value = sell_buy[wk->type][1] - sell_buy[wk->type][0];
            //double current_cost = frameID >= 13760 ? (time_buy + time_sell) / (17.7 * value) : (time_buy + time_sell);
                
            //4 5 6缺原料的台优先考虑合成：
            double Q3 = 1.0;
            if(Workspace::workspaces[target2Id]->is_first()){
                Q3 = 0.84 * Q3;
            }


            //4 5 6被访问次数最少的工作台类型，加权值


            double Q1 = 1.0;
            if (num_of_seven && num_of_4 && num_of_5 && num_of_6 && frameID < close_do7_frame) {
                int min_visit = *min_element(Workspace::visit_times, Workspace::visit_times + 3);
                int max_visit = *max_element(Workspace::visit_times, Workspace::visit_times + 3);
                if (Workspace::visit_times[Workspace::workspaces[target2Id]->type - 4] == min_visit) {
                    if ((max_visit - min_visit) == 1) {
                        Q1 = 0.9;
                    }
                    if ((max_visit - min_visit) >= 2) {
                        Q1 = Q1 - 0.4 * (max_visit - min_visit - 1);
                    }
                    // cerr<<"man_visit="<<max_visit<<endl;
                    // cerr<<"min_visit="<<min_visit<<endl;
                }
            }
            

            // 用于7号台之间竞争456 
            double Q2 = 1.0;
            if (num_of_seven && num_of_4 && num_of_5 && num_of_6 && frameID < close_do7_frame) {
                if (Workspace::workspaces[target2Id]->type == 7) {
                    //原料格差多少个合成 
                    if (!Workspace::workspaces[target2Id]->is_full_reserved_and_resource()) {//防止gettime > remain_time就去送
                        //此时预定位还没置位，原料格会差 1 2 3个才能进入合成
                        if (Workspace::workspaces[target2Id]->num_of_ready_resouce_of_seven() == 3 &&
                            Workspace::workspaces[target2Id]->preduct_state == 0 &&
                            Workspace::workspaces[target2Id]->remain_preduct_time <= (time_buy + time_sell) &&
                            Workspace::workspaces[target2Id]->remain_preduct_time != -1) {
                            Q2 = 1.0;
                        }
                        else {
                            Q2 = 1.0 - 0.4 * Workspace::workspaces[target2Id]->num_of_ready_resouce_of_seven();
                            double q = (Workspace::workspaces[target2Id]->remain_preduct_time + 1.1) / 1000.0;//可调参数
                            Q2 = Q2 * (0.4 + q * 0.6);//尽量让7之间得竞争不影响到其他买卖点的cost

                        }

                    }

                }
            }
            // 买7的权重 效果不好
            // double Q7 = 1.0;
            // double Q8 = 1.0;
            // if(wk->type == 7){
            //     if(wk->preduct_state == 1 ){//防止没生产好，提前预约的
            //         if(wk->remain_preduct_time != -1){
            //             double q = (wk->remain_preduct_time + 1.1)/1000.0;//可调参数
            //             Q7 =  q ; 
            //         }
            //         Q8 = 0.33*(3 - wk->num_of_ready_resouce_of_seven());
            //     }
            // }
            
            


            

            //卖点为7时，根据原料格状态，调整456（为买点、卖点）的优先级，//效果不行
            // if(Workspace::workspaces[target2Id]->type == 7){
            //     Workspace::workspaces[target2Id]->update_impact_factor();
            // }
            // double Q = 1.0;
            // for (int i = 0; i < 3; i++)
            // {
            //     if(wk->type == (4+i) || Workspace::workspaces[target2Id]->type == (4+i)){
            //         Q = Workspace::impact_factor[i];
            //     }
            // }

            //靠近障碍物的工作台的权值 //没效果
            // double Q4 = 1.0;
            // if(Workspace::workspaces[target2Id]->is_near_barrier()){
            //     Q4 = 1.0;
            // }

            //边角工作台的权值 //会叠加Q4的权值 //没效果
            // double Q5 = 1.0;
            // if(Workspace::workspaces[target2Id]->is_corner_workspace()){
            //     Q5 = 0.9;
            // }
            //买点 边角工作台的权值 //会叠加Q4的权值 //没效果
            // double Q6 = 1.0;
            // if(wk->is_corner_workspace()){
            //     Q6 = 0.9;
            // }

            //TODO:卖123时，考虑上456号台距离7号台的距离

            double time_c = func(time_sell); // 时间系数
            double value = time_c * sell_buy[wk->type][1] - sell_buy[wk->type][0];

            current_sum_dist = frameID >= close_do7_frame ? (time_buy + time_sell) / (17.7 * value) : (time_buy + time_sell);
            
            if(mapID == 2 || mapID == 1){//TODO:
                current_sum_dist *= Q1*Q2*Q3;
            }
            

            

            if(cost_sum_dist > current_sum_dist){
                cost_sum_dist = current_sum_dist;
                target1 = wk->id;
                target2 = target2Id;
                // path_buy.clear();
                // path_sell.clear();
                path_buy = temp_buy;
                path_sell = allpaths[wk->index_x][wk->index_y][target2Id][1];
            }
        }
        // 找不到其他地方卖，最后再考虑卖给9号工作台
        // 如果无暂存记录，且有9号
        if (target2 == -1 && nine_workspace_is)
        {
            for (Workspace *nine_wk : (Workspace::workspaces))
            {
                if(allpaths_valid[wk->index_x][wk->index_y][nine_wk->id][1]){
                        //可达
                }else{
                    continue;
                }
                if (nine_wk->type != 9)
                    continue;
                //if (nine_wk->reserved[wk->type] == 1)
                //    continue;

                int cnt = 0;
                for (auto i : nine_wk->reserved) cnt += i.second;
                if (cnt > 2) continue;

                double dist2 = allpaths_dis[wk->index_x][wk->index_y][nine_wk->id][1];
                time_sell_nine = get_time_by_dist(dist2);
                // 增加判断游戏结束时候能不能卖出去，否则跳过
                if ((double)frameID + time_buy + time_sell_nine + end_extern_time > 15000.)
                {
                    continue;
                }

                if (time_buy + time_sell_nine < nine_cost)
                {
                    nine_cost = time_buy + time_sell_nine;
                    target1 = wk->id;
                    target2_nine = nine_wk->id;
                    path_buy = temp_buy;
                    path_sell_nine = allpaths[wk->index_x][wk->index_y][nine_wk->id][1];
                    record_time_buy = time_buy;
                    record_time_sell = time_sell;
                }
            }
        }
    }
    
    

    if(target2 != -1){
        task.push(make_pair(target1,path_buy));
        task.push(make_pair(target2,path_sell));

        //维护预定位数据结构
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target1]->type] = 1;

        //维护visit_times数组
        if (((Workspace::workspaces)[target2]->type >= 4) && (Workspace::workspaces)[target2]->type <= 6) {
            Workspace::visit_times[Workspace::workspaces[target2]->type - 4]++;
        }

        //捎货
        // if (((Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target2]->type] == 0) &&
        //     ((Workspace::workspaces)[target2]->preduct_state == 1 ||
        //         (record_time_buy + record_time_sell  > (Workspace::workspaces)[target2]->remain_preduct_time &&
        //             (Workspace::workspaces)[target2]->remain_preduct_time != -1))) {
        //     int target3 = -1;
        //     vector<Point> path3,path3_buy;
        //     for (int target3Id : (Workspace::workspaces)[target2]->next_workspace) {
        //         int temp_index_x,temp_index_y;
        //         get_index_by_coordinate(Workspace::workspaces[target2]->x,Workspace::workspaces[target2]->y,temp_index_x,temp_index_y);
        //         vector<Point> temp_sao = allpaths[temp_index_x][temp_index_y][target2][0];
        //         double temp_dist,dist1;
        //         if(temp_sao.empty()){
        //             if(a_star(Point(temp_index_x,temp_index_y),target2,0,temp_sao,dist1)){
        //                 allpaths[temp_index_x][temp_index_y][target2][0] = temp_sao;
        //                 allpaths_dis[temp_index_x][temp_index_y][target2][0] = temp_dist;
        //             }else{
        //                 continue;
        //             }
        //         }
        //         double dist3 = allpaths_dis[temp_index_x][temp_index_y][target3Id][1];
        //         double t3 = get_time_by_dist(dist3);
        //         if (((Workspace::workspaces)[target3Id]->reserved[(Workspace::workspaces)[target2]->type] == 0)
        //             && ((Workspace::workspaces)[target3Id]->resource_state[(Workspace::workspaces)[target2]->type] == 0 ||
        //                 ((Workspace::workspaces)[target3Id]->is_full_resource() &&
        //                     (Workspace::workspaces)[target3Id]->remain_preduct_time != -1 &&
        //                     (Workspace::workspaces)[target3Id]->remain_preduct_time <= record_time_buy + record_time_sell + t3 &&
        //                     (Workspace::workspaces)[target3Id]->preduct_state == 0))) {
        //                         // 增加判断游戏结束时候能不能卖出去，否则跳过
        //                         if ((double)frameID + record_time_buy + record_time_sell + t3 + end_extern_time > 15000.0)
        //                         {
        //                             is_handle_collision = 0;
        //                             continue;
        //                         }else{
        //                             is_handle_collision =  1;
        //                         }
        //             if (t3 < record_time_sao) { 
        //                 record_time_sao = t3; 
        //                 target3 = target3Id;
        //                 path3_buy = temp_sao;
        //                 path3 = allpaths[temp_index_x][temp_index_y][target3][1];
        //             }
        //         }
        //     }
        //     if (target3>=0 && (Workspace::workspaces)[target3]->type == 9) {
        //         int cnt = 0;
        //         for (auto i : (Workspace::workspaces)[target3]->reserved) {
        //             cnt += i.second;
        //         }
        //         if (cnt > 2) { target3=-1; }
        //     }
        //     if (target3 != -1) {
        //         task.push(make_pair(target2,path3_buy));
        //         task.push(make_pair(target3,path3));
        //         (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target2]->type] = 1;
        //         //int flag = (Workspace::workspaces)[target3]->type == 8 || (Workspace::workspaces)[target3]->type == 9 ? 0 : 1;
        //         (Workspace::workspaces)[target3]->reserved[(Workspace::workspaces)[target2]->type] = 1;
        //     }
        // }
        
    }
    else if (target2_nine != -1)
    {
        task.push(make_pair(target1,path_buy));
        task.push(make_pair(target2_nine,path_sell_nine));
        // 添加预定
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        (Workspace::workspaces)[target2_nine]->reserved[(Workspace::workspaces)[target1]->type] = 1;
    }
    else{
        //找不到
        // cerr<<id<<"seek fail.."<<endl;
    }
}

void Robot::seek_parallar(){
    if(!task.empty()){
        return;
    }
    Workspace::init_impact_factor();

    //接任务
    double cost_sum_dist = 100000.0;
    double current_sum_dist;
    double nine_cost = 100000.0;
    vector<Point> path_buy;
    vector<Point> path_sell;
    vector<Point> path_sell_nine;
    int target1 = -1;
    int target2 = -1;
    int target2_nine = -1;
    vector<Point> temp_buy;
    double dist_buy = 100000.0;
    double dist_sell = 100000.0;
    double dist_sell_nine = 100000.0;
    double time_buy,time_sell,time_sell_nine;
    double record_time_buy,record_time_sell;
    double record_time_sao = 100000.0;

    //临时变量，机器人当前点的索引
    // int temp_index_x, temp_index_y;
    // get_index_by_coordinate(x,y,temp_index_x,temp_index_y);

    for(Workspace* wk : Workspace::workspaces){//买点
        // 不可达点跳过
        if(!workspace_is_reachable[wk->id]){
            // cerr<<wk->id << "unreachable"<<endl;
            continue;
        } 
        if(wk->reserved[wk->type] == 1){//被预定
            // cerr<<"has reserved..skip."<<endl;
            continue;
        }
        temp_buy =  allpaths[index_x][index_y][wk->id][0];
        // for(Point ele: allpaths[temp_index_x][temp_index_y][wk->id][0]){
        //     cerr<<ele.index_x<<" "<<ele.index_y<<endl;
        // }
        // cerr<<wk->id <<"buy_path_size "<<temp_buy.size() <<endl;
        if(temp_buy.empty()){
            if(a_star(Point(index_x,index_y),wk->id,0,temp_buy,dist_buy)){
                allpaths[index_x][index_y][wk->id][0] = temp_buy;
                allpaths_dis[index_x][index_y][wk->id][0] = dist_buy;
            }else{
                //cerr<<wk->id <<"has no road ,skiped... " <<endl;
                continue;
            }
        }
        dist_buy = allpaths_dis[index_x][index_y][wk->id][0];//dist赋值

        time_buy = get_time_by_dist(dist_buy);
        if (wk->preduct_state == 1 || ( (time_buy > wk->remain_preduct_time) && (wk->remain_preduct_time != -1) )){//没生产好
            //没问题
        }else{
            continue;
        }

        for (int target2Id : wk->next_workspace){
            if(allpaths_valid[wk->index_x][wk->index_y][target2Id][1]){
                //可达
            }else{
                continue;
            }
            if(Workspace::workspaces[target2Id]->reserved[wk->type] == 1)//该原料格被预定
            {
                continue;
            }
            dist_sell = allpaths_dis[wk->index_x][wk->index_y][target2Id][1];//dist赋值，工作点之间，必定有值
            // cerr<<dist_sell<<endl;

            time_sell = get_time_by_dist(dist_sell);
            // 原料格空或 原料格满且生产时间小于到达时间
            if ( (Workspace::workspaces[target2Id]->resource_state[wk->type] == 0) ||
                (Workspace::workspaces[target2Id]->is_full_resource() &&              // 原料格满
                    Workspace::workspaces[target2Id]->remain_preduct_time != -1 &&       // 正在生产
                    (Workspace::workspaces[target2Id]->remain_preduct_time < (time_buy + time_sell)) && // 到达的时候生产完成
                    Workspace::workspaces[target2Id]->preduct_state == 0) )            // 产品格必须为空（否则可能一直阻塞生产不出来）
            {
                //放宽卖点被选择条件后，该卖点没问题
            }else{
                continue;
            }

            // 计算和比较花销选择最小的情况
            double time_c = func(time_sell); // 时间系数
            double value = sell_buy[wk->type][1] - sell_buy[wk->type][0];
            double current_cost =  (time_buy + time_sell);
            
            //
            double Q = 1.0;
            if(id <= 2){//0 1 2机器人各侧重于一个电台
                Workspace::impact_factor[id] = 0.2;//0 1 2机器人主要负责于某一个类型的机器人
                if(wk->type == (4+id) || Workspace::workspaces[target2Id]->type == (4+id)){
                    Q = Workspace::impact_factor[id];
                }
                //若7缺，优先给7送原料
                if(Workspace::workspaces[target2Id] -> type == 7 &&
                    (Workspace::workspaces[target2Id] -> reserved[wk->type] == 0 &&
                    Workspace::workspaces[target2Id] -> resource_state[wk->type] == 0)){
                    Q = Q*0.5;
                }
            }
            if(id == 3){//3号机器人打辅助
                // cerr<<id<<endl;
                if(wk->type == 7){
                    Q = 0.6;//7生产好了优先去拿
                    // cerr<<"1st if error"<<endl;
                }
                int min_visit = *min_element(Workspace::visit_times, Workspace::visit_times + 3);
                // cerr<<"hello min_visit"<<endl;
                if(Workspace::workspaces[target2Id]->type == 4 ||
                    Workspace::workspaces[target2Id]->type == 5 ||
                    Workspace::workspaces[target2Id]->type == 6){
                        // cerr<<"2st if ..."<<endl;
                        if(Workspace::visit_times[Workspace::workspaces[target2Id]->type - 4] == min_visit){
                            // cerr<<"3st if ..."<<endl;
                            Q = 0.8;
                        }
                    }else{
                        // cerr<<"2st if"<<endl;
                    }
            }
            //TODO:多个7，给谁送
            // double Q1 = 1.0;
            // if(Workspace::workspaces[target2Id]->type == 7){
            //     //剩余生产时间
            //     double q;
            //     q = (Workspace::workspaces[target2Id]->remain_preduct_time + 1.1)/1000.0;//可调参数
            //     Q1 = Q1*0.6 + q*0.4;
                // //原料格差多少个合成 
                // if(!Workspace::workspaces[target2Id]->is_full_resource()){//防止gettime > remain_time就去送
                //     //此时预定位还没置位，原料格会差 1 2 3个 才能进入合成
                //     Q1 = 1.0 - 0.2 *(3 - Workspace::workspaces[target2Id]->num_of_ready_resouce_of_seven());
                // }
            //     //原料格状态
            //     //预定状态
            // }


            current_sum_dist = dist_buy + dist_sell;
            current_sum_dist *= Q ;

            if(cost_sum_dist > current_sum_dist){
                cost_sum_dist = current_sum_dist;
                target1 = wk->id;
                target2 = target2Id;
                path_buy.clear();
                path_sell.clear();
                path_buy = temp_buy;
                path_sell = allpaths[wk->index_x][wk->index_y][target2Id][1];
            }
        }
        // 找不到其他地方卖，最后再考虑卖给9号工作台
        // 如果无暂存记录，且有9号
        if (target2 == -1 && nine_workspace_is)
        {
            for (Workspace *nine_wk : (Workspace::workspaces))
            {
                if(allpaths_valid[wk->index_x][wk->index_y][nine_wk->id][1]){
                        //可达
                }else{
                    continue;
                }
                if (nine_wk->type != 9)
                    continue;
                //if (nine_wk->reserved[wk->type] == 1)
                //    continue;



                int cnt = 0;
                for (auto i : nine_wk->reserved) cnt += i.second;
                if (cnt > 2) continue;

                double dist2 = allpaths_dis[wk->index_x][wk->index_y][nine_wk->id][1];
                time_sell_nine = get_time_by_dist(dist2);
                // 增加判断游戏结束时候能不能卖出去，否则跳过
                if ((double)frameID + time_buy + time_sell_nine + 10.0 > 15000.0)
                {
                    continue;
                }

                if (time_buy + time_sell_nine < nine_cost)
                {
                    nine_cost = time_buy + time_sell_nine;
                    target1 = wk->id;
                    target2_nine = nine_wk->id;
                    path_buy = temp_buy;
                    path_sell_nine = allpaths[wk->index_x][wk->index_y][nine_wk->id][1];
                    record_time_buy = time_buy;
                    record_time_sell = time_sell;
                }
            }
        }
    }
    
    

    if(target2 != -1){
        task.push(make_pair(target1,path_buy));
        task.push(make_pair(target2,path_sell));

        //维护预定位数据结构
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target1]->type] = 1;

        //记录
        from_where = Point(index_x, index_y);
        time_err_record = frameID;

        //维护visit_times数组
        if (((Workspace::workspaces)[target2]->type >= 4) && (Workspace::workspaces)[target2]->type <= 6) {
            Workspace::visit_times[Workspace::workspaces[target2]->type - 4]++;
        }

        //捎货
        // if (((Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target2]->type] == 0) &&
        //     ((Workspace::workspaces)[target2]->preduct_state == 1 ||
        //         (record_time_buy + record_time_sell  > (Workspace::workspaces)[target2]->remain_preduct_time &&
        //             (Workspace::workspaces)[target2]->remain_preduct_time != -1))) {
        //     int target3 = -1;
        //     vector<Point> path3,path3_buy;
        //     for (int target3Id : (Workspace::workspaces)[target2]->next_workspace) {
        //         int temp_index_x,temp_index_y;
        //         get_index_by_coordinate(Workspace::workspaces[target2]->x,Workspace::workspaces[target2]->y,temp_index_x,temp_index_y);
        //         vector<Point> temp_sao = allpaths[temp_index_x][temp_index_y][target2][0];
        //         double temp_dist,dist1;
        //         if(temp_sao.empty()){
        //             if(a_star(Point(temp_index_x,temp_index_y),target2,0,temp_sao,dist1)){
        //                 allpaths[temp_index_x][temp_index_y][target2][0] = temp_sao;
        //                 allpaths_dis[temp_index_x][temp_index_y][target2][0] = temp_dist;
        //             }else{
        //                 continue;
        //             }
        //         }
        //         double dist3 = allpaths_dis[temp_index_x][temp_index_y][target3][1];
        //         double t3 = get_time_by_dist(dist3);
        //         if (((Workspace::workspaces)[target3Id]->reserved[(Workspace::workspaces)[target2]->type] == 0)
        //             && ((Workspace::workspaces)[target3Id]->resource_state[(Workspace::workspaces)[target2]->type] == 0 ||
        //                 ((Workspace::workspaces)[target3Id]->is_full_resource() &&
        //                     (Workspace::workspaces)[target3Id]->remain_preduct_time != -1 &&
        //                     (Workspace::workspaces)[target3Id]->remain_preduct_time <= record_time_buy + record_time_sell + t3 &&
        //                     (Workspace::workspaces)[target3Id]->preduct_state == 0))) {
        //                         // 增加判断游戏结束时候能不能卖出去，否则跳过
        //                         if ((double)frameID + record_time_buy + record_time_sell + t3 + 10.0 > 15000.0)
        //                         {
        //                             continue;
        //                         }
        //             if (t3 < record_time_sao) { 
        //                 record_time_sao = t3; 
        //                 target3 = target3Id;
        //                 path3_buy = temp_sao;
        //                 path3 = allpaths[temp_index_x][temp_index_y][target3][1];
        //             }
        //         }
        //     }
        //     if (target3>=0 && (Workspace::workspaces)[target3]->type == 9) {
        //         int cnt = 0;
        //         for (auto i : (Workspace::workspaces)[target3]->reserved) {
        //             cnt += i.second;
        //         }
        //         if (cnt > 2) { target3=-1; }
        //     }
        //     if (target3 != -1) {
        //         task.push(make_pair(target2,path3_buy));
        //         task.push(make_pair(target3,path3));
        //         (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target2]->type] = 1;
        //         //int flag = (Workspace::workspaces)[target3]->type == 8 || (Workspace::workspaces)[target3]->type == 9 ? 0 : 1;
        //         (Workspace::workspaces)[target3]->reserved[(Workspace::workspaces)[target2]->type] = 1;
        //     }
        // }
        
    }
    else if (target2_nine != -1)
    {
        task.push(make_pair(target1,path_buy));
        task.push(make_pair(target2_nine,path_sell_nine));
        // 添加预定
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        (Workspace::workspaces)[target2_nine]->reserved[(Workspace::workspaces)[target1]->type] = 1;

        from_where = Point(index_x, index_y);
        time_err_record = frameID;
    }
    else{
        //找不到
        // cerr<<id<<"seek fail.."<<endl;
    }
}


double Robot::func(double holdFrame)
{ // 计算时间系数,不考虑碰撞
    return (1 - sqrt(1 - pow(1 - holdFrame / 15000., 2))) * (1 - 0.8) + 0.8;
}

void Robot::buy()
{ // 机器人的买行为

    //超过多少帧之后，不进行buy操作 TODO:
    if (task.empty())
        return; // 为空队列就不需要买

    int next_workspace = task.front().first;                             // 当前目标工作台

    // //判断工作台附近是否有障碍物， 有 则需要特判！
    // //缩小买的范围，避免买货变大卡死！!~
    // //如果是边角工作台，不可以特判，否则买货变大可能会被卡住
    // if(is_near_workspace == next_workspace && Workspace::workspaces[next_workspace]->is_near_barrier()
    //     && !Workspace::workspaces[next_workspace]->is_corner_workspace()){
    //     int robot_temp_index_x,robot_temp_index_y;
    //     get_index_by_coordinate(x,y,robot_temp_index_x,robot_temp_index_y);
    //     if(robot_temp_index_x != Workspace::workspaces[next_workspace]->index_x || 
    //         robot_temp_index_y != Workspace::workspaces[next_workspace]->index_y){
    //             return;
    //         }
    // }

    if (is_near_workspace != -1 && load_id == 0 &&                   // 附近有工作台 且 当前未携带物品
        is_near_workspace == next_workspace &&                       // 到达的这个工作台是目标工作台
        (Workspace::workspaces)[next_workspace]->preduct_state == 1) // 到达的这个工作台已生产出来
    {
        // cerr<<id<<"do buy()..."<<endl;
        printf("buy %d\n", id);

        // cerr<<"buy time = "<<frameID<<endl;

        /* 任务出队 只能买了才能出队 */
        task.pop();

        /* 买了之后更新 目标工作台 产品格 被预定的状态为0 */
        Workspace::workspaces[next_workspace]->reserved[Workspace::workspaces[next_workspace]->type] = 0;
        /* 买了之后更新 目标工作台 产品状态为 空 */
        Workspace::workspaces[next_workspace]->preduct_state = 0;

        /* 判断是否在工作台同一个位置，不在就重新a_star */
        if(index_x != Workspace::workspaces[next_workspace]->index_x ||
            index_y != Workspace::workspaces[next_workspace]->index_y) {
                // cerr<<" not in one site "<<endl;
                vector<Point> path;
                double dis;
                if(a_star(Point(index_x, index_y), task.front().first, 1, path, dis))
                {
                    task.front().second = path;
                }
                else{
                    cerr<< " cant find "<<endl;
                }
            }
        
        if (allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][0] > 1e-3) {
            if(id == 0)
            // cerr << frameID - time_err_record - get_time_by_dist(allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][0]) << endl;
            time_err_cnt++;
            time_err += frameID - time_err_record - get_time_by_dist(allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][0]);
            //cerr << "ok"<< get_time_by_dist(allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][0]) << endl;
            from_where = Point(Workspace::workspaces[next_workspace]->index_x, Workspace::workspaces[next_workspace]->index_y);
        }
        time_err_record = frameID;
    }
    return;
}

void Robot::sell()
{ // 机器人的售出行为
    if (task.empty())
        return; // 为空队列就不需要卖

    int next_workspace = task.front().first; // 当前任务队列的队首，返回该工作台的id
    if (is_near_workspace != -1 && load_id > 0 &&
        is_near_workspace == next_workspace &&                                // 附近工作台是目标工作台
        (Workspace::workspaces)[next_workspace]->resource_state[load_id] == 0 // 目标工作台原料格为空
    )
    {

        // cerr<<id<<"do sell()..."<<endl;
        printf("sell %d\n", id);

        /* 任务出队 */
        task.pop();
        
        /* 卖了之后更新 目标工作台的 原料格(被机器人携带的产品) 被预定的状态 */
        Workspace::workspaces[next_workspace]->reserved[load_id] = 0;
        /* 卖了之后更新 目标工作台的 原料格(被机器人携带的产品) 装填的状态 */
        Workspace::workspaces[next_workspace]->resource_state[load_id] = 1;

        if (allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][1] > 1e-3) {
            if (id == 0)
            // cerr << frameID - time_err_record - get_time_by_dist(allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][1]) << endl;
            time_err_cnt++;
            time_err += frameID - time_err_record - get_time_by_dist(allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][1]);
            //cerr << "ok" << get_time_by_dist(allpaths_dis[from_where.index_x][from_where.index_y][next_workspace][0]) << endl;
            from_where = Point(Workspace::workspaces[next_workspace]->index_x, Workspace::workspaces[next_workspace]->index_y);
        }
        time_err_record = frameID;


        // sao
        // if (task.empty()) {
        //     double record_t = 1e9;
        //     vector<Point> path,path_buy;
        //     path_buy.push_back(Point(Workspace::workspaces[next_workspace]->index_x,Workspace::workspaces[next_workspace]->index_y));
        //     if (((Workspace::workspaces)[next_workspace]->reserved[(Workspace::workspaces)[next_workspace]->type] == 0) &&
        //         ((Workspace::workspaces)[next_workspace]->preduct_state == 1 ||
        //             (50.00 > (Workspace::workspaces)[next_workspace]->remain_preduct_time &&
        //                 (Workspace::workspaces)[next_workspace]->remain_preduct_time != -1))) {
        //         int target = -1;
        //         double dist;
        //         for (int targetId : (Workspace::workspaces)[next_workspace]->next_workspace) {
        //             int temp_index_x,temp_index_y;
        //             temp_index_x = Workspace::workspaces[next_workspace]->index_x;
        //             temp_index_y = Workspace::workspaces[next_workspace]->index_y;
                    
        //             vector<Point> temp_path =  allpaths[temp_index_x][temp_index_y][targetId][1];
        //             // cerr << temp_path.size() <<endl;
        //             dist = allpaths_dis[temp_index_x][temp_index_y][targetId][1];
        //             double t = get_time_by_dist(dist);

        //             if (((Workspace::workspaces)[targetId]->reserved[(Workspace::workspaces)[next_workspace]->type] == 0)
        //                 && ((Workspace::workspaces)[targetId]->resource_state[(Workspace::workspaces)[next_workspace]->type] == 0 ||
        //                     ((Workspace::workspaces)[targetId]->is_full_resource() &&
        //                         (Workspace::workspaces)[targetId]->remain_preduct_time != -1 &&
        //                         (Workspace::workspaces)[targetId]->remain_preduct_time <= t &&
        //                         (Workspace::workspaces)[targetId]->preduct_state == 0))) {
        //                 // 增加判断游戏结束时候能不能卖出去，否则跳过
        //                 if ((double)frameID + t + 10.0 > 15000.0)
        //                 {
        //                     continue;
        //                 }
        //                 if (t < record_t) { 
        //                     record_t = t; 
        //                     target = targetId;
        //                     path = temp_path;
        //                 }
        //             }
        //         }
        //         if (target != -1) {
        //             task.push(make_pair(next_workspace,path_buy));
        //             task.push(make_pair(target,path));
        //             (Workspace::workspaces)[next_workspace]->reserved[(Workspace::workspaces)[next_workspace]->type] = 1;
        //             //int flag = (Workspace::workspaces)[target3]->type == 8 || (Workspace::workspaces)[target3]->type == 9 ? 0 : 1;
        //             (Workspace::workspaces)[target]->reserved[(Workspace::workspaces)[next_workspace]->type] = 1;
        //         }
        //     }
        // }
    }
}

/*--------------------------------------------------------move接口0314-----------------------------------*/

/*-----------------------------------------------------------mov stable 最稳定----------------------------------*/
void Robot::move_stable()
{
    if (fabs(v[0]) <= 1e-7 && fabs(v[1]) <= 1e-7 && frameID < 100)
    {
        printf("forward %d %f\n", id, 3.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }

    bool nearface = 0;
    for (Robot* rot : avoid_robots) {
        if (sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < (r + rot->r + 0.5)) {
            nearface = 1;
        }
    }
    if (is_avoiding && avoid_path.size() == 0 && nearface) {
        printf("forward %d %f\n", id, -2.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }

    // 非避让情况下卡死就后退一下，解决图3的卡死4工作台
    // if(frameID % 200 == 0 || frameID == 1){
    //     last_position2 = Point(index_x,index_y);
    // }
    // int up,down,left,right;
    // bool is_in_danger = in_danger(index_x, index_y, load_id, Point(-1,-1), up, down, left, right);
    // if(frameID % 200 > 50 && !is_avoiding && is_near_workspace && is_in_danger){ // 非避让情况下
    //     if(last_position2.index_x == index_x && last_position2.index_y == index_y){
    //         // cerr<<"hello kasi..."<<endl;
    //         printf("forward %d %f\n", id, -2.0);
    //         printf("rotate %d %f\n", id, PI);
    //         return;
    //     }    
    // }

    if (frameID % 300 == 0 || frameID == 1) {
        last_position = Point(index_x, index_y);
    }
    if (frameID % 300 > 50) {
        if (last_position.index_x == index_x && last_position.index_y == index_y) {
            // cerr<<"hello kasi..."<<endl;
            printf("forward %d %f\n", id, 6.0);
            printf("rotate %d %f\n", id, PI);
            double temp;
            if(!avoid_path.empty()){
                a_star(Point(index_x,index_y),avoid_Point,load_id,avoid_path,temp);
            }else if(!task.empty() && !task.front().second.empty()){
                a_star(Point(index_x,index_y),task.front().first,load_id,task.front().second,temp);
            }
            return;
        }
    }

    for (Robot* rot : robots) {
        if (rot->id == id) {
            continue;
        }
        if (sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < (r + rot->r + 1e-3)) {
            if (find(rot->avoid_robots.begin(), rot->avoid_robots.end(), this) != rot->avoid_robots.end()) {
                printf("forward %d %f\n", id, -2.0);
                printf("rotate %d %f\n", id, 0.0);
                return;
            }
        }
    }




    double theta, movcnt_theta, kpv_theta;
    double delete_boundary = 0.5;
    double cur_v = sqrt(v[0] * v[0] + v[1] * v[1]);

    if (cur_v > 5.) {
        delete_boundary = 1.;
    }
    else if (cur_v > 4.) {
        delete_boundary = 0.8;
    }


    if (is_avoiding) {
        if (avoid_path.size() > 1)
        {
            Point p1 = avoid_path[0];
            if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                // back_path.insert(back_path.begin(),avoid_path[0]);
                avoid_path.erase(avoid_path.begin());
            }
        }
    }
    else {
        if (is_backing) {
            if (back_path.size() > 0) {//back_path不用留点
                Point p1 = back_path[0];
                if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                    back_path.erase(back_path.begin());
                }
            }
        }
        else {
            if (task.empty() || task.front().second.empty())
            {
                printf("forward %d %f\n", id, 0.0);
                printf("rotate %d %f\n", id, 0.0);
                return;
            }
            if (task.front().second.size() > 1) {
                Point p1 = task.front().second[0];
                double delete_dot = (p1.x - x) * v[0] + (p1.y - y) * v[1];
                if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                    task.front().second.erase(task.front().second.begin());
                }
            }
        }
    }


    double targetx, targety;
    if (is_avoiding && avoid_path.size()) {
        targetx = avoid_path[0].x;
        targety = avoid_path[0].y;
    }
    else if (is_backing && back_path.size()) {
        targetx = back_path[0].x;
        targety = back_path[0].y;
    }
    else {
        if (task.empty() || task.front().second.empty())
        {
            printf("forward %d %f\n", id, 0.0);
            printf("rotate %d %f\n", id, 0.0);
            return;
        }
        targetx = task.front().second[0].x;
        targety = task.front().second[0].y;
    }


    pair<double, double> dis_vec(targetx - x, targety - y);
    double dis_vec_m = sqrt(dis_vec.first * dis_vec.first + dis_vec.second * dis_vec.second);
    if (dis_vec_m < 1e-3) {
        theta = -rotation;
    }
    else {
        theta = atan2(dis_vec.second, dis_vec.first) - rotation;
    }
    if (theta > PI) {
        theta = theta - 2 * PI;
    }
    else if (theta < -PI) {
        theta = theta + 2 * PI;
    }



    bool will_in_wall = 0;
    int idx = 0, idy = 0;
    double vv = sqrt(v[0] * v[0] + v[1] * v[1]);
    double willx = x + vv * cos(rotation) * 10.0 / 50.0;
    double willy = y + vv * sin(rotation) * 10.0 / 50.0;
    get_index_by_coordinate(willx, willy, idx, idy);
    //cerr << idx << " " << idy << " " << graph[idx][idy] << endl;;

    if (graph[idx][idy] == '#')
        will_in_wall = 1;


    if (!will_in_wall && is_handle_collision == 1 && 0)//TODO:没开碰撞
    {
        for (int cid = 0; cid < 4; cid++)
        {
            if (cid == id) continue;
            pair<double, double> BO(robots[cid]->x - x, robots[cid]->y - y);
            double br = robots[cid]->load_id ? 0.53 : 0.45, ar = load_id ? 0.53 : 0.45;
            double BR = br + ar;


            double BOO_dis = sqrt(pow(BO.first, 2) + pow(BO.second, 2));

            double t = pow(BOO_dis, 2) - pow(BR, 2);
            t = t <= 1e-4 ? 0.01 : t;
            double tangencyO_dis = sqrt(t);
            pair<double, double> BO_unit_v(BO.first / BOO_dis, BO.second / BOO_dis);
            double tt = BR / BOO_dis;
            tt = tt > 1.0 ? 1.0 : tt;
            tt = tt < -1.0 ? -1.0 : tt;
            double angle = asin(tt);
            pair<double, double> tangency1(BO_unit_v.first * cos(angle) - BO_unit_v.second * sin(angle),
                BO_unit_v.first * sin(angle) + BO_unit_v.second * cos(angle));
            pair<double, double> tangency2(BO_unit_v.first * cos(-angle) - BO_unit_v.second * sin(-angle),
                BO_unit_v.first * sin(-angle) + BO_unit_v.second * cos(-angle));
            tangency1.first *= tangencyO_dis;
            tangency1.second *= tangencyO_dis;
            tangency2.first *= tangencyO_dis;
            tangency2.second *= tangencyO_dis;

            pair<double, double> relative_v(v[0] - robots[cid]->v[0], v[1] - robots[cid]->v[1]);
            double relative_v_m = sqrt(pow(relative_v.first, 2) + pow(relative_v.second, 2));
            double crossp1 = relative_v.first * tangency1.second - relative_v.second * tangency1.first;
            double crossp2 = relative_v.first * tangency2.second - relative_v.second * tangency2.first;
            double cfs = 0.0, crs = 0.0;
            if (crossp1 * crossp2 < 0) {
                pair<double, double> BOv(relative_v.first - BO.first, relative_v.second - BO.second);
                pair<double, double> notBO(-BO.first, -BO.second);
                double dotBOv = BOv.first * notBO.first + BOv.second * notBO.second;
                double BOv_m = sqrt(pow(BOv.first, 2) + pow(BOv.second, 2));
                if (dotBOv < 0 || (dotBOv > 0 && BOv_m < BR)) {
                    double u1_m = fabs(crossp1) / tangencyO_dis;
                    double u2_m = fabs(crossp2) / tangencyO_dis;
                    if (u1_m < u2_m) {
                        tt = u1_m / relative_v_m;
                        tt = tt > 1.0 ? 1.0 : tt;
                        tt = tt < -1.0 ? -1.0 : tt;
                        crs = 277.0 * asin(tt) * crossp1 / fabs(crossp1);
                        pair<double, double> u1(0.0, 0.0);
                        if (1) {
                            u1.first = -tangency1.second / tangencyO_dis;
                            u1.second = tangency1.first / tangencyO_dis;
                        }
                        u1.first *= u1_m;
                        u1.second *= u1_m;
                        pair<double, double> vhope1(v[0] + u1.first / 2.0, v[1] + u1.second / 2.0);
                        cfs = 50.0 * sqrt(pow(vhope1.first, 2) + pow(vhope1.second, 2));
                    }
                    else {
                        tt = u2_m / relative_v_m;
                        tt = tt > 1.0 ? 1.0 : tt;
                        tt = tt < -1.0 ? -1.0 : tt;
                        crs = 277.0 * asin(tt) * crossp2 / fabs(crossp2);
                        pair<double, double> u2(0.0, 0.0);
                        if (1) {
                            u2.first = tangency1.second / tangencyO_dis;
                            u2.second = -tangency1.first / tangencyO_dis;
                        }
                        u2.first *= u2_m;
                        u2.second *= u2_m;
                        pair<double, double> vhope2(v[0] + u2.first / 2.0, v[1] + u2.second / 2.0);
                        cfs = 50.0 * sqrt(pow(vhope2.first, 2) + pow(vhope2.second, 2));
                    }
                    if (dotBOv > 0 && BOv_m < BR) {
                        /* double u3_m = BR - BOv_m;
                         if (u3_m < min(u1_m, u2_m)) {
                             pair<double, double> u3(relative_v.first - BO.first, relative_v.second - BO.second);
                             u3.first *= u3_m / BOv_m;
                             u3.second *= u3_m / BOv_m;
                             pair<double, double> vhope3(v[0] + u3.first / 2.0, v[1] + u3.second / 2.0);
                             double vhope3_m = sqrt(pow(vhope3.first, 2) + pow(vhope3.second, 2));
                             cfs = vhope3_m;
                             double vh3_rv_dot = vhope3.first * relative_v.first + vhope3.second * relative_v.second;
                             double rv_vh3_crossp = relative_v.first * vhope3.second - relative_v.second * vhope3.first;
                             relative_v_m = relative_v_m < 1e-4 ? 1.0 : relative_v_m;
                             vhope3_m = vhope3_m < 1e-4 ? 1.0 : vhope3_m;
                             tt = vh3_rv_dot / (relative_v_m * vhope3_m);
                             tt = tt < -1.0 ? -1.0 : tt;
                             tt = tt > 1.0 ? 1.0 : tt;
                             crs = 7.7 * acos(tt) * rv_vh3_crossp / fabs(rv_vh3_crossp);
                         }*/
                    }
                    printf("forward %d %f\n", id, cfs);
                    printf("rotate %d %f\n", id, crs);
                    return;
                }
            }
        }
    }

    double base_fs, fs, rs;

    if (fabs(theta) > PI / 4. && load_id) {
        base_fs = 2.;
    }
    else if (fabs(theta) > PI / 6. && load_id) {
        base_fs = 7.;
    }
    else if (fabs(theta) > PI / 4. && !load_id) {
        base_fs = 6.;
    }
    else if (load_id) {
        base_fs = 17.;
    }
    else {
        base_fs = 23.;
    }

    rs = 7.0 * theta;
    fs = will_in_wall ? -2.00 : 1. / (fabs(rs) + 0.35) * base_fs;

    double theta_highspeed_boundary = 0.15;
    if (!load_id) theta_highspeed_boundary = 0.2;

    if (fabs(theta) < theta_highspeed_boundary) {
        rs = 0.;
        fs = 6.;
    }




    int path_size;

    if (is_avoiding) {
        path_size = avoid_path.size();
    }
    else if (is_backing && back_path.size()) {
        path_size = back_path.size();
    }
    else {
        path_size = task.front().second.size();
    }
    if (dis_vec_m < 1.0 && path_size == 1)
    {
        fs = 1.70 * dis_vec_m;
        if (dis_vec_m < 0.3 && is_avoiding) {
            fs = 0.0;
        }
    }

    printf("forward %d %f\n", id, fs);
    printf("rotate %d %f\n", id, rs);


    if (id == 0) {
        //cerr << fs << endl;
    }
}

void Robot::move_stable3()
{
    if (fabs(v[0]) <= 1e-7 && fabs(v[1]) <= 1e-7 && frameID < 100)
    {
        printf("forward %d %f\n", id, 3.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }

    bool nearface = 0;
    for (Robot* rot : avoid_robots) {
        if (sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < (r + rot->r + 0.5)) {
            nearface = 1;
        }
    }
    if (is_avoiding && avoid_path.size() == 0 && nearface) {
        printf("forward %d %f\n", id, -2.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }

    // 非避让情况下卡死就后退一下，解决图3的卡死4工作台
    // if(frameID % 200 == 0 || frameID == 1){
    //     last_position2 = Point(index_x,index_y);
    // }
    // int up,down,left,right;
    // bool is_in_danger = in_danger(index_x, index_y, load_id, Point(-1,-1), up, down, left, right);
    // if(frameID % 200 > 50 && !is_avoiding && is_near_workspace && is_in_danger){ // 非避让情况下
    //     if(last_position2.index_x == index_x && last_position2.index_y == index_y){
    //         // cerr<<"hello kasi..."<<endl;
    //         printf("forward %d %f\n", id, -2.0);
    //         printf("rotate %d %f\n", id, PI);
    //         return;
    //     }    
    // }

    if (frameID % 200 == 0 || frameID == 1) {
        last_position = Point(index_x, index_y);
    }
    if (frameID % 200 > 60) {
        if (last_position.index_x == index_x && last_position.index_y == index_y) {
            // cerr<<"hello kasi..."<<endl;
            printf("forward %d %f\n", id, 6.0);
            printf("rotate %d %f\n", id, PI);
            double temp;
            if(!avoid_path.empty()){
                a_star(Point(index_x,index_y),avoid_Point,load_id,avoid_path,temp);
            }else if(!task.empty() && !task.front().second.empty()){
                a_star(Point(index_x,index_y),task.front().first,load_id,task.front().second,temp);
            }
            return;
        }
    }

    for (Robot* rot : robots) {
        if (rot->id == id) {
            continue;
        }
        if (sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < (r + rot->r + 0.1)) {
            if (find(rot->avoid_robots.begin(), rot->avoid_robots.end(), this) != rot->avoid_robots.end()) {
                printf("forward %d %f\n", id, -2.0);
                printf("rotate %d %f\n", id, 0.0);
                return;
            }
        }
    }




    double theta, movcnt_theta, kpv_theta;
    double delete_boundary = 0.5;
    double cur_v = sqrt(v[0] * v[0] + v[1] * v[1]);

    if (cur_v > 5.) {
        delete_boundary = 1.;
    }
    else if (cur_v > 4.) {
        delete_boundary = 0.8;
    }


    if (is_avoiding) {
        if (avoid_path.size() > 1)
        {
            Point p1 = avoid_path[0];
            if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                // back_path.insert(back_path.begin(),avoid_path[0]);
                avoid_path.erase(avoid_path.begin());
            }
        }
    }
    else {
        if (is_backing) {
            if (back_path.size() > 0) {//back_path不用留点
                Point p1 = back_path[0];
                if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                    back_path.erase(back_path.begin());
                }
            }
        }
        else {
            if (task.empty() || task.front().second.empty())
            {
                printf("forward %d %f\n", id, 0.0);
                printf("rotate %d %f\n", id, 0.0);
                return;
            }
            if (task.front().second.size() > 1) {
                Point p1 = task.front().second[0];
                double delete_dot = (p1.x - x) * v[0] + (p1.y - y) * v[1];
                if (sqrt(pow(x - p1.x, 2) + pow(y - p1.y, 2)) < delete_boundary) {
                    task.front().second.erase(task.front().second.begin());
                }
            }
        }
    }


    double targetx, targety;
    if (is_avoiding && avoid_path.size()) {
        targetx = avoid_path[0].x;
        targety = avoid_path[0].y;
    }
    else if (is_backing && back_path.size()) {
        targetx = back_path[0].x;
        targety = back_path[0].y;
    }
    else {
        if (task.empty() || task.front().second.empty())
        {
            printf("forward %d %f\n", id, 0.0);
            printf("rotate %d %f\n", id, 0.0);
            return;
        }
        targetx = task.front().second[0].x;
        targety = task.front().second[0].y;
    }


    pair<double, double> dis_vec(targetx - x, targety - y);
    double dis_vec_m = sqrt(dis_vec.first * dis_vec.first + dis_vec.second * dis_vec.second);
    if (dis_vec_m < 1e-3) {
        theta = -rotation;
    }
    else {
        theta = atan2(dis_vec.second, dis_vec.first) - rotation;
    }
    if (theta > PI) {
        theta = theta - 2 * PI;
    }
    else if (theta < -PI) {
        theta = theta + 2 * PI;
    }



    bool will_in_wall = 0;
    int idx = 0, idy = 0;
    double vv = sqrt(v[0] * v[0] + v[1] * v[1]);
    double willx = x + vv * cos(rotation) * 10.0 / 50.0;
    double willy = y + vv * sin(rotation) * 10.0 / 50.0;
    get_index_by_coordinate(willx, willy, idx, idy);
    //cerr << idx << " " << idy << " " << graph[idx][idy] << endl;;

    if (graph[idx][idy] == '#')
        will_in_wall = 1;


    if (!will_in_wall && is_handle_collision == 1 && 0)//TODO:没开碰撞
    {
        for (int cid = 0; cid < 4; cid++)
        {
            if (cid == id) continue;
            pair<double, double> BO(robots[cid]->x - x, robots[cid]->y - y);
            double br = robots[cid]->load_id ? 0.53 : 0.45, ar = load_id ? 0.53 : 0.45;
            double BR = br + ar;


            double BOO_dis = sqrt(pow(BO.first, 2) + pow(BO.second, 2));

            double t = pow(BOO_dis, 2) - pow(BR, 2);
            t = t <= 1e-4 ? 0.01 : t;
            double tangencyO_dis = sqrt(t);
            pair<double, double> BO_unit_v(BO.first / BOO_dis, BO.second / BOO_dis);
            double tt = BR / BOO_dis;
            tt = tt > 1.0 ? 1.0 : tt;
            tt = tt < -1.0 ? -1.0 : tt;
            double angle = asin(tt);
            pair<double, double> tangency1(BO_unit_v.first * cos(angle) - BO_unit_v.second * sin(angle),
                BO_unit_v.first * sin(angle) + BO_unit_v.second * cos(angle));
            pair<double, double> tangency2(BO_unit_v.first * cos(-angle) - BO_unit_v.second * sin(-angle),
                BO_unit_v.first * sin(-angle) + BO_unit_v.second * cos(-angle));
            tangency1.first *= tangencyO_dis;
            tangency1.second *= tangencyO_dis;
            tangency2.first *= tangencyO_dis;
            tangency2.second *= tangencyO_dis;

            pair<double, double> relative_v(v[0] - robots[cid]->v[0], v[1] - robots[cid]->v[1]);
            double relative_v_m = sqrt(pow(relative_v.first, 2) + pow(relative_v.second, 2));
            double crossp1 = relative_v.first * tangency1.second - relative_v.second * tangency1.first;
            double crossp2 = relative_v.first * tangency2.second - relative_v.second * tangency2.first;
            double cfs = 0.0, crs = 0.0;
            if (crossp1 * crossp2 < 0) {
                pair<double, double> BOv(relative_v.first - BO.first, relative_v.second - BO.second);
                pair<double, double> notBO(-BO.first, -BO.second);
                double dotBOv = BOv.first * notBO.first + BOv.second * notBO.second;
                double BOv_m = sqrt(pow(BOv.first, 2) + pow(BOv.second, 2));
                if (dotBOv < 0 || (dotBOv > 0 && BOv_m < BR)) {
                    double u1_m = fabs(crossp1) / tangencyO_dis;
                    double u2_m = fabs(crossp2) / tangencyO_dis;
                    if (u1_m < u2_m) {
                        tt = u1_m / relative_v_m;
                        tt = tt > 1.0 ? 1.0 : tt;
                        tt = tt < -1.0 ? -1.0 : tt;
                        crs = 277.0 * asin(tt) * crossp1 / fabs(crossp1);
                        pair<double, double> u1(0.0, 0.0);
                        if (1) {
                            u1.first = -tangency1.second / tangencyO_dis;
                            u1.second = tangency1.first / tangencyO_dis;
                        }
                        u1.first *= u1_m;
                        u1.second *= u1_m;
                        pair<double, double> vhope1(v[0] + u1.first / 2.0, v[1] + u1.second / 2.0);
                        cfs = 50.0 * sqrt(pow(vhope1.first, 2) + pow(vhope1.second, 2));
                    }
                    else {
                        tt = u2_m / relative_v_m;
                        tt = tt > 1.0 ? 1.0 : tt;
                        tt = tt < -1.0 ? -1.0 : tt;
                        crs = 277.0 * asin(tt) * crossp2 / fabs(crossp2);
                        pair<double, double> u2(0.0, 0.0);
                        if (1) {
                            u2.first = tangency1.second / tangencyO_dis;
                            u2.second = -tangency1.first / tangencyO_dis;
                        }
                        u2.first *= u2_m;
                        u2.second *= u2_m;
                        pair<double, double> vhope2(v[0] + u2.first / 2.0, v[1] + u2.second / 2.0);
                        cfs = 50.0 * sqrt(pow(vhope2.first, 2) + pow(vhope2.second, 2));
                    }
                    if (dotBOv > 0 && BOv_m < BR) {
                        /* double u3_m = BR - BOv_m;
                         if (u3_m < min(u1_m, u2_m)) {
                             pair<double, double> u3(relative_v.first - BO.first, relative_v.second - BO.second);
                             u3.first *= u3_m / BOv_m;
                             u3.second *= u3_m / BOv_m;
                             pair<double, double> vhope3(v[0] + u3.first / 2.0, v[1] + u3.second / 2.0);
                             double vhope3_m = sqrt(pow(vhope3.first, 2) + pow(vhope3.second, 2));
                             cfs = vhope3_m;
                             double vh3_rv_dot = vhope3.first * relative_v.first + vhope3.second * relative_v.second;
                             double rv_vh3_crossp = relative_v.first * vhope3.second - relative_v.second * vhope3.first;
                             relative_v_m = relative_v_m < 1e-4 ? 1.0 : relative_v_m;
                             vhope3_m = vhope3_m < 1e-4 ? 1.0 : vhope3_m;
                             tt = vh3_rv_dot / (relative_v_m * vhope3_m);
                             tt = tt < -1.0 ? -1.0 : tt;
                             tt = tt > 1.0 ? 1.0 : tt;
                             crs = 7.7 * acos(tt) * rv_vh3_crossp / fabs(rv_vh3_crossp);
                         }*/
                    }
                    printf("forward %d %f\n", id, cfs);
                    printf("rotate %d %f\n", id, crs);
                    return;
                }
            }
        }
    }

    double base_fs, fs, rs;

    if (fabs(theta) > PI / 4. && load_id) {
        base_fs = 2.;
    }
    else if (fabs(theta) > PI / 6. && load_id) {
        base_fs = 7.;
    }
    else if (fabs(theta) > PI / 4. && !load_id) {
        base_fs = 6.;
    }
    else if (load_id) {
        base_fs = 17.;
    }
    else {
        base_fs = 23.;
    }

    rs = 7.0 * theta;
    fs = will_in_wall ? -2.00 : 1. / (fabs(rs) + 0.35) * base_fs;

    double theta_highspeed_boundary = 0.15;
    if (!load_id) theta_highspeed_boundary = 0.2;

    if (fabs(theta) < theta_highspeed_boundary) {
        rs = 0.;
        fs = 6.;
    }




    int path_size;

    if (is_avoiding) {
        path_size = avoid_path.size();
    }
    else if (is_backing && back_path.size()) {
        path_size = back_path.size();
    }
    else {
        path_size = task.front().second.size();
    }
    if (dis_vec_m < 1.0 && path_size == 1)
    {
        fs = 1.70 * dis_vec_m;
        if (dis_vec_m < 0.3 && is_avoiding) {
            fs = 0.0;
        }
    }

    printf("forward %d %f\n", id, fs);
    printf("rotate %d %f\n", id, rs);


    if (id == 0) {
        //cerr << fs << endl;
    }
}
/*-----------------------------------------------------------mov stable 最稳定----------------------------------*/
/*----------------------------------------------------------v1.0---------------------------------------------*/

double Robot::get_time_by_dist(double dist){
    double err = time_err_cnt < 1e-3 ? 0. : time_err / time_err_cnt;
    // return dist / 4.7 * 50.0 + err*0.3;
    return dist / total_average_speed * 50.0 ;//TODO:加入dist\ave_v

    //cerr << err << endl;
}
/*----------------------------------------------------------v1.0---------------------------------------------*/


bool point_near(Point p1,Point p2){
    if(abs(p1.index_x - p2.index_x) + abs(p1.index_y - p2.index_y) <= 2)//曼哈顿距离<=2视为邻格
    {
        return true;
    }
    return false;
}
bool Robot::low_priority(Robot* rot){
    //没任务的机器人优先级最高
    if(task.empty() || task.front().second.empty()) {
        return true;
    }
    if (rot->task.empty()) {
        return false;
    }
    if(!load_id){
        if(rot->load_id){
            return true;
        }else if(id < rot->id){
            return true;
        }
    }else{
        if(rot->load_id && (id < rot->id)){
            return true;
        }
    }
    return false;
}

bool Robot::near_other_robot(Robot* rot){//用于工作台附近是否可以优先级反转
        // if (sqrt(pow(rot->x - x, 2) + pow(rot->y - y, 2)) < (r + rot->r + 8.0)) {
        //     return 1;//TODO:
        // }
        return 0;
}

//TODO: 没有任务了被人供着走，不让路？
void Robot::conflict_avoidance(){
    //  第2阶段.维护avoid_robots的vector数组，检测该数组中机器人是否离开
    //TODO: 不带货时，避让接触，重新seek？
    if(!avoid_robots.empty()){
        // 维护路径寻路 backpath
        double temp;
        if(!task.empty() && !task.front().second.empty())//TODO:
            a_star(Point(index_x,index_y),task.front().first,load_id,task.front().second,temp);
        //如果task为空，无任务要执行，就继续躲避其他机器人

        vector<Point> new_path;
        new_path = back_path; // =next_path - avoid_path
        new_path.insert(new_path.begin(),Point(index_x,index_y));//防止机器人贴脸
        if(!task.empty()){
            new_path.insert(new_path.end(),task.front().second.begin(),task.front().second.end());
        }
        // 判断是否还冲突
        for(int i = 0; i < avoid_robots.size();i++){
            if(task.empty()){//如果该机器人没任务，继续避让其他机器人
                break;
            }
            // 首先比较优先级是否倒转就可以不避让
            // TODO: && !near_other_robot(avoid_robots[i])
            if(!low_priority(avoid_robots[i]) && !near_other_robot(avoid_robots[i]) ){
                avoid_robots.erase(avoid_robots.begin()+i); // 去除此机器人
                if(avoid_robots.empty()){ // 当无避让时，可以清空避让路径，回到正常任务队列开始运行
                    avoid_path.clear();
                    is_avoiding = 0;
                    if(!back_path.empty()){
                        is_backing = 1;
                    }
                    next_path = new_path;
                }
                continue;
            }

            bool remove = 1;
            bool priority_reverse = 0;

            for(int j = 0; j < new_path.size();j++){ //当前机器人重启任务的路径 :返回的路+任务路径
                if(j >= avoid_range){ 
                    break; 
                }
                for(int k=0; k < avoid_robots[i]->next_path.size(); k++){//要避让的机器人接下来要走的路
                    if(k >= avoid_range){
                        break;
                    }
                    if(abs(j-k) > 5){ // 考虑冲突格与俩个机器人的距离
                        continue;
                    }
                    if(point_near(new_path[j],avoid_robots[i]->next_path[k])){
                        
                        remove = 0;
                    }
                }
            }
            //TODO: 加强碰撞策略！
            if(remove == 1){ // 当前避让机器人已没有冲突路径
                avoid_robots.erase(avoid_robots.begin()+i); // 去除此机器人
                if(avoid_robots.empty()){ // 当无避让时，可以清空避让路径，回到正常任务队列开始运行
                    avoid_path.clear();
                    is_avoiding = 0;
                    if(!back_path.empty()){
                        is_backing = 1;
                    }
                    next_path = new_path;
                }
            }
        }
    }
    //  1.每帧做冲突检测
    //没任务时，躲避其他机器人
    // if(task.empty()){//TODO:
    //     for(Robot* rot : robots){
    //         if(rot->id == id){
    //             continue;
    //         }
    //         //如果已经在冲突机器人数组了，continue;
    //         if(find(avoid_robots.begin(),avoid_robots.end(),rot) != avoid_robots.end()){
    //             continue;
    //         }
    //         // 如果自己已在对方避免碰撞数组，跳过
    //         if(find(rot->avoid_robots.begin(), rot->avoid_robots.end(), this) != rot->avoid_robots.end()) {
    //             continue; 
    //         }
            
    //         avoid_robots.push_back(rot);
    //         is_avoiding = 1;
    //         //记录冲突路径格子
    //         confilic_point[rot] = Point(index_x,index_y);
    //         continue;
    //     }
    // }
    for(int i=0;i<next_path.size();i++){//当前机器人的next_path
        if(i >= avoid_range){//遍历next_path的至多前10个点
            break;
        }
        for(Robot* rot : robots){//遍历其他机器人，检测是否冲突
            if(rot->id == id){
                continue;
            }
            //如果已经在冲突机器人数组了，continue;
            if(find(avoid_robots.begin(),avoid_robots.end(),rot) != avoid_robots.end()){
                continue;
            }
            // 如果自己已在对方避免碰撞数组，跳过
            if(find(rot->avoid_robots.begin(), rot->avoid_robots.end(), this) != rot->avoid_robots.end()) {
                continue; 
            }
           
            for(int j=0;j < rot->next_path.size();j++){
                if(j >= avoid_range){
                    break;
                }
                if(abs(j-i) > 5) continue; // 相差点过多应该就不会相撞了
                if(point_near(next_path[i],rot->next_path[j]) || 
                    point_near(Point(index_x,index_y),rot->next_path[j])){//路径存在邻格
                    if(low_priority(rot))//优先级低的点为主动避让点 //TODO: 谁避让快，谁为主动避让点
                    {
                        avoid_robots.push_back(rot);
                        is_avoiding = 1;

                        // //记录冲突时当前机器人所在格子
                        // back_target = Point(index_x,index_y);

                        //记录冲突路径格子
                        confilic_point[rot] = rot->next_path[j];
                        // cerr<<id<< " rot + avoid_rots.."<<endl;
                    }
                }
            }
        }
    }

    //  3.避让处理逻辑 更新avoid_path
    if (!avoid_robots.empty()) 
    {
        // cerr<<id<<"robot handle conflic"<<endl;
        char graph_temp[width][width];
        for (Robot *rot : avoid_robots)
        {
            // 先存graph
            for (int j = -1; j <= 1; j++)
            { // 冲突机器人本身变成障碍物
                for (int k = -1; k <= 1; k++)
                {
                    //冲突机器人周围一圈
                    //考虑边界越界：
                    if(in_range(rot->index_x + j,rot->index_y + k)){

                    }else{
                        continue;
                    }
                    graph_temp[rot->index_x + j][rot->index_y + k] = graph[rot->index_x + j][rot->index_y + k];
                }
            }
            for (int i = 0; i < rot->next_path.size(); i++)
            {
                if (i >= avoid_range)
                {
                    //只考虑next_path路径前十个点
                    break;
                }
                int x = rot->next_path[i].index_x;
                int y = rot->next_path[i].index_y;
                // 先存
                for (int j = -1; j <= 1; j++)
                { // 冲突机器人接下来路径上的点变为障碍物
                    for (int k = -1; k <= 1; k++)
                    {
                        //路径一圈变为障碍物
                        //考虑边界 越界：
                        if(in_range(x + j,y + k)){

                        }else{
                            continue;
                        }
                        graph_temp[x + j][y + k] = graph[x + j][y + k];
                    }
                }
            }
        }
        // 后graph更新障碍物区域
        for (Robot *rot : avoid_robots)
        {
            for (int j = -1; j <= 1; j++)
            { // 冲突机器人本身及其邻格一圈变成障碍物
                for (int k = -1; k <= 1; k++)
                {
                    //冲突机器人周围一圈
                    //考虑边界越界：
                    if(in_range(rot->index_x + j,rot->index_y + k)){

                    }else{
                        continue;
                    }
                    graph[rot->index_x + j][rot->index_y + k] = '#';
                }
            }

            bool nei_flag = 0; 
            //判断俩个机器人是否邻格2圈
            for (int j = -2; j <= 2; j++)
            {
                for (int k = -2; k <= 2; k++)
                {
                    if(rot->index_x + j == index_x && rot->index_y + k == index_y){
                        nei_flag = 1;
                        break;
                    }
                }
            }
            if(nei_flag) // 表示两个机器人是挨着了已经
            {
                for (int j = -1; j <= 1; j++)
                {
                    for (int k = -1; k <= 1; k++)
                    {
                        if(k == 0 && j== 0) continue; // 对方机器人位置设为障碍物保留
                        graph[rot->index_x+j][rot->index_y+k] = graph_temp[rot->index_x+j][rot->index_y+k];
                    }
                }

                continue; // 挨着就不需要延伸路径了
            }

            for (int i = 0; i < rot->next_path.size(); i++)
            {
                // 最多只将障碍物加到冲突点或者最远扫描的10个格子
                if(confilic_point[rot].hash_f() == rot->next_path[i].hash_f()) break; 
                if (i >= avoid_range)
                {
                    break;
                }
                int x = rot->next_path[i].index_x;
                int y = rot->next_path[i].index_y;
                if(x == index_x && y == index_y) break; // 路径添加障碍物扫到当前机器人就停止，放置断避让机器人路径
                for (int j = -1; j <= 1; j++)
                {
                    for (int k = -1; k <= 1; k++)
                    {
                        graph[x + j][y + k] = '#';
                    }
                }
            }
            
        }
        // BFS找 合适的点 去躲避
        Point target;
        queue<Point> q;
        unordered_map<int,bool> visit;
        q.push(Point(index_x,index_y));
        bool find_des = 0; // bfs是否找到避让点 
        while(!q.empty()){
            Point cur = q.front();

            //判断该点是否满足条件
            bool cur_ok = 1;
            // 不带货 四周无障碍物就可选为为避让点
            if(!load_id){
                for(int i = -1; i <= 1;i++){
                    for(int j = -1;j<=1;j++){
                        if(!in_range(cur.index_x + i,cur.index_y + j) || 
                            graph[cur.index_x + i][cur.index_y + j] == '#'){
                            cur_ok = 0;
                        }
                    }
                }
                //加严格选点条件
                if(!in_range(cur.index_x + 2,cur.index_y) ||
                    graph[cur.index_x + 2][cur.index_y] == '#'){
                    cur_ok = 0;
                }
                if(!in_range(cur.index_x - 2,cur.index_y) ||
                    graph[cur.index_x - 2][cur.index_y] == '#'){
                    cur_ok = 0;
                }
                if(!in_range(cur.index_x, cur.index_y + 2) ||
                    graph[cur.index_x][cur.index_y + 2] == '#'){
                    cur_ok = 0;
                }
                if(!in_range(cur.index_x,cur.index_y - 2) ||
                    graph[cur.index_x][cur.index_y - 2] == '#'){
                    cur_ok = 0;
                }
            }else{ // 带货 周围1圈+上下左右2距离 内不可以成为避让点
                for(int i = -1; i <= 1;i++){
                    for(int j = -1;j<=1;j++){
                        if(!in_range(cur.index_x + i,cur.index_y + j) ||
                            graph[cur.index_x + i][cur.index_y + j] == '#'){
                            cur_ok = 0;
                        }
                    }
                }
                if(!in_range(cur.index_x + 2,cur.index_y) ||
                    graph[cur.index_x + 2][cur.index_y] == '#'){
                    cur_ok = 0;
                }
                if(!in_range(cur.index_x - 2,cur.index_y) ||
                    graph[cur.index_x - 2][cur.index_y] == '#'){
                    cur_ok = 0;
                }
                if(!in_range(cur.index_x, cur.index_y + 2) ||
                    graph[cur.index_x][cur.index_y + 2] == '#'){
                    cur_ok = 0;
                }
                if(!in_range(cur.index_x,cur.index_y - 2) ||
                    graph[cur.index_x][cur.index_y - 2] == '#'){
                    cur_ok = 0;
                }
            }

            // 被冲突的机器人未来路径都设置无效点，不可以作为避让点，
            // 路径延申的点附近曼哈顿距离2以内不可选作避让终点

            // 不可以一直延申，如果要延申，可以延申20 30格子 ,不然过于严格 干扰其他
            for(Robot* rot : avoid_robots){
                for(int i = 0;i < rot->next_path.size(); i++){
                    if(i >= 25){
                        break;
                    }
                    if(point_near(cur,rot->next_path[i])){
                        cur_ok = 0;
                    }
                }
            }

            if(cur_ok == 1){ // 找到避让点，跳出去a_star寻路
                target = cur;
                avoid_Point = target;
                find_des = 1;
                break;
            }

            q.pop();
            for(int i = -1; i <= 1;i++){
                for(int j = -1;j<=1;j++){
                    Point temp_p(cur.index_x +i,cur.index_y+j);
                    int temp = temp_p.hash_f();
                    if(visit.count(temp)){//map中存在该元素
                        continue;
                    }
                    // 超出地图范围跳过
                    if(!in_range(temp_p.index_x, temp_p.index_y)) continue;
                    // 判断该点是否可以加入
                    int up,down,left,right;
                    if (in_danger(temp_p.index_x, temp_p.index_y, load_id, Point(-1,-1), up, down, left, right))
                        continue; 
                    // 加入队列
                    visit[temp] = 1;
                    q.push(temp_p);
                }
            }
        }
        // a_star寻路
        if(find_des){
                //更新avoid_path
                double temp;
                // cerr<<id<<"robote BFS failed "<<index_x<<" "<<index_y<<" avoid rot"<<endl;
                // cerr<<avoid_robots[0]->index_x<<" "<<avoid_robots[0]->index_y<<endl;
                // cerr<<id<<"robote a_star"<<
                a_star(Point(index_x,index_y),target,load_id,avoid_path,temp);
                // cerr<<endl;
        }else{
            // cerr<<id<<"robote BFS failed "<<index_x<<" "<<index_y<<" avoid rot"<<endl;
            // cerr<<avoid_robots[0]->index_x<<" "<<avoid_robots[0]->index_y<<endl;
            // for(int j = -2;j <= 2; j++){  
            //     for(int k = -2;k <= 2; k++){
            //         cerr<<graph[index_x+j][index_y+k];
            //     }
            //     cerr<<endl;
            // }
            // cerr<<endl;
        }
        //还原地图
        for(Robot* rot : avoid_robots){
            //冲突机器人接下来路径上的点变为障碍物
            for(int i = 0; i < rot->next_path.size(); i ++){
                if(i >= avoid_range){
                    break;
                }
                int x = rot->next_path[i].index_x;
                int y = rot->next_path[i].index_y;
                for(int j = -1;j <= 1; j++){  
                    for(int k = -1;k <= 1; k++){
                        if(in_range(x + j,y + k)){

                        }else{
                            continue;
                        }
                        graph[x+j][y+k] = graph_temp[x+j][y+k];
                    }
                }
            }
            //冲突机器人本身变成障碍物
            int x = rot->index_x;
            int y = rot->index_y;
            for(int j = -1;j <= 1; j++){  
                for(int k = -1;k <= 1; k++){
                    if(in_range(rot->index_x + j,rot->index_y + k)){

                    }else{
                        continue;
                    }
                    graph[x+j][y+k] = graph_temp[x+j][y+k];
                }
            }
        }
    }
}

void Robot::update_all_robots_next_path(){
    if(avoid_path.empty()){
        is_avoiding = 0;
        if(!back_path.empty()){
            is_backing = 1;
        }else{
            is_backing = 0;
        }
    }
    else{
        is_avoiding = 1;
    }
    // 接下来的路径是由冲突转移路+返回路径+任务路径
    next_path = avoid_path;
    next_path.insert(next_path.end(), back_path.begin(),back_path.end());
    if(!task.empty() && !task.front().second.empty()){
        next_path.insert(next_path.end(), task.front().second.begin(),task.front().second.end());
    }
    
}


