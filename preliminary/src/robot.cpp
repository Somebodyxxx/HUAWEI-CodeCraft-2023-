#include "robot.h"
#include "workspace.h"
#include "main.h"

#include <math.h>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>

using namespace std;

int Robot::money = 200000;

vector<Robot *> Robot::robots;

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
}

void Robot::seek()
{
    // 任务队列不为空，继续执行，不用考虑接任务
    if (!task.empty())
        return;
    Workspace::init_impact_factor();
    double cost = 100000; // 最小花费cost初始化
    // 暂存记录，选择最优的入队结点的id
    int target1 = -1;
    int target2 = -1;

    // 4-8均找不到卖的地方，用于卖到最近9号台的处理
    double nine_cost = 100000;
    int nine_target1 = -1;
    int nine_target2 = -1;

    double t1 = 0.0, t2 = 0.0, t3 = 0.0, record_t1 = 0.0, record_t2 = 0.0, record_t3 = 1e9;

    // 第一层循环，遍历所有可以“买货物”的工作台
    for (Workspace *wk : (Workspace::workspaces))
    {
        // 买货物——产品格被预定，剪枝
        if (wk->reserved[wk->type] == 1)
            continue;

        // 第一目标点有产品，且到达时间>产品生产剩余时间，可以进一步考虑
        t1 = get_time1(wk->id);

        // TODO: 扩大到可以等待c帧生产时间
        if (wk->preduct_state == 1 ||                                          // 有产品
            (t1 > (wk->remain_preduct_time) && wk->remain_preduct_time != -1)) // 无产品但是正在生产并且可以到达时完成生产
        {
            // 考虑买了该产品之后第二步去哪卖,遍历有向图中该买点的所有下一“可达结点”
            for (int target2Id : wk->next_workspace)
            {
                // 卖货物，原料格被预定，不考虑
                if ((Workspace::workspaces)[target2Id]->reserved[wk->type] == 1 &&
                (Workspace::workspaces)[target2Id]->type != 9)
                    continue;
                if ((Workspace::workspaces)[target2Id]->type == 9) {
                    int cnt = 0;
                    for (auto i : (Workspace::workspaces)[target2Id]->reserved) { 
                        cnt+=i.second;
                    }
                    if (cnt > 2) { continue; } // TODO:
                }

                //优化，使得卖给4 5 6平均。
                if(num_of_seven){
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
               

                // 第一目标点有产品，且到达时间>产品生产剩余时间，可以进一步考虑
                t2 = get_time2(wk->id, target2Id, t1);
                // 增加判断游戏结束时候能不能卖出去，否则跳过
                if ((double)frameID + t1 + t2 + 10.0 > 9000.0){
                    is_handle_collision = 0;
                    continue;
                }else{
                    is_handle_collision = 1;
                }
                    

                // 原料格空或 原料格满且生产时间小于到达时间
                if ((Workspace::workspaces)[target2Id]->resource_state[wk->type] == 0 ||
                    ((Workspace::workspaces)[target2Id]->is_full_resource() &&              // 原料格满
                     (Workspace::workspaces)[target2Id]->remain_preduct_time != -1 &&       // 正在生产
                     (Workspace::workspaces)[target2Id]->remain_preduct_time <= t1 + t2 && // 到达的时候生产完成
                        (Workspace::workspaces)[target2Id]->preduct_state == 0))             // 产品格必须为空（否则可能一直阻塞生产不出来）
                {
                    // 计算和比较花销选择最小的情况
                    double time_c = func(t2); // 时间系数
                    double value = sell_buy[wk->type][1] - sell_buy[wk->type][0];
                    //TODO: 根据原料格状态调整贪心策略，贪时间 or 价值。
                    double current_cost = frameID >= 7760 ? (t1 + t2)/(17.7*value) : (t1+t2);
                    // if((Workspace::workspaces)[target2Id]->remain_preduct_time < 200.0 &&
                    //     (Workspace::workspaces)[target2Id]->resource_state_o != 0){
                    //         // current_cost *= 0.9;
                    //         if((Workspace::workspaces)[target2Id]->type == 7){
                    //             current_cost *= 0.8;
                    //         }
                    //     }
                    
                    //4 5 6缺原料的台优先考虑合成：
                    double Q3 = 1.0;
                    if(Workspace::workspaces[target2Id]->is_first()){
                        Q3 = 0.85 * Q3;
                        if(num_of_seven == 2){//面向地图编程
                            Q3 = 0.91;
                        }
                    }

                    // 7号台缺啥，若买点是，加权重！用于7号台之间竞争456 
                    double Q2 = 1.0;
                    if(Workspace::workspaces[target2Id]->type == 7){
                        if(Workspace::workspaces[target2Id]->resource_state[wk->type] == 0 &&
                            Workspace::workspaces[target2Id]->reserved[wk->type] == 0){
                                double q = (Workspace::workspaces[target2Id]->remain_preduct_time + 2.0)/1000.0;//可调参数
                                Q2 = (0.85 + q*0.15) * Q2;//取0.1，尽量让7之间得竞争不影响到其他买卖点的cost
                            }
                    }

                    //卖点为7时，根据原料格状态，调整456（为买点、卖点）的优先级，
                    if(Workspace::workspaces[target2Id]->type == 7){
                        Workspace::workspaces[target2Id]->update_impact_factor();
                        // cerr<<"update..";
                        // for (int i = 0; i < 3; i++)
                        // {
                        //     cerr<<Workspace::impact_factor[i]<<" ";
                        // }
                        // cerr<<endl;
                    }
                    double Q = 1.0;
                    for (int i = 0; i < 3; i++)
                    {
                        if(wk->type == (4+i) || Workspace::workspaces[target2Id]->type == (4+i)){
                            Q = Workspace::impact_factor[i];
                        }
                    }

                    current_cost = current_cost * Q * Q2 * Q3;
                    

                    if (current_cost < cost)
                    { // 找最小
                        cost = current_cost;
                        target1 = wk->id;
                        target2 = target2Id;
                        record_t1 = t1;
                        record_t2 = t2;
                    }
                }
            }
            // 找不到其他地方卖，最后再考虑卖给9号工作台
            // 如果无暂存记录，且有9号
            if (target2 == -1 && nine_workspace_is)
            {
                for (Workspace *nine_wk : (Workspace::workspaces))
                {
                    if (nine_wk->type != 9)
                        continue;
                    //if (nine_wk->reserved[wk->type] == 1)
                    //    continue;

                    int cnt = 0;
                    for (auto i : nine_wk->reserved) cnt += i.second;
                    if (cnt > 2) continue;

                    t2 = get_time2(wk->id, nine_wk->id, t1);
                    // 增加判断游戏结束时候能不能卖出去，否则跳过
                    if ((double)frameID + t1 + t2 + 10.0 > 9000.0)
                    {
                        is_handle_collision = 0;
                        continue;
                    }else{
                        is_handle_collision =  1;
                    }

                    if (t1 + t2 < nine_cost)
                    {
                        nine_cost = t1 + t2;
                        nine_target1 = wk->id;
                        nine_target2 = nine_wk->id;
                    }
                }
            }
        }
    }

    // 若找到最近的目标，添加到任务队列
    if (target2 != -1)
    {
        task.push(target1);
        task.push(target2);
        // cerr<<(Workspace::workspaces)[target1]->type<<"+"<<(Workspace::workspaces)[target2]->type<<endl;
        // 添加预定
        (Workspace::workspaces)[target1]->reserved[(Workspace::workspaces)[target1]->type] = 1;
        (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target1]->type] = 1;

        if (((Workspace::workspaces)[target2]->type >= 4) && (Workspace::workspaces)[target2]->type <= 6) {
            Workspace::visit_times[Workspace::workspaces[target2]->type - 4]++;
        }

        if (((Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target2]->type] == 0) &&
            ((Workspace::workspaces)[target2]->preduct_state == 1 ||
                (record_t1 + record_t2  > (Workspace::workspaces)[target2]->remain_preduct_time &&
                    (Workspace::workspaces)[target2]->remain_preduct_time != -1))) {
            int target3 = -1;
            for (int target3Id : (Workspace::workspaces)[target2]->next_workspace) {
                t3 = get_time3(target1, target2, target3Id, 0.0);
                if (((Workspace::workspaces)[target3Id]->reserved[(Workspace::workspaces)[target2]->type] == 0)
                    && ((Workspace::workspaces)[target3Id]->resource_state[(Workspace::workspaces)[target2]->type] == 0 ||
                        ((Workspace::workspaces)[target3Id]->is_full_resource() &&
                            (Workspace::workspaces)[target3Id]->remain_preduct_time != -1 &&
                            (Workspace::workspaces)[target3Id]->remain_preduct_time <= t1 + t2 + t3 &&
                            (Workspace::workspaces)[target3Id]->preduct_state == 0))) {
                                // 增加判断游戏结束时候能不能卖出去，否则跳过
                                if ((double)frameID + record_t1 + record_t2 + t3 + 10.0 > 9000.0)
                                {
                                    is_handle_collision = 0;
                                    continue;
                                }else{
                                    is_handle_collision =  1;
                                }
                    if (t3 < record_t3) { record_t3 = t3; target3 = target3Id; }
                }
            }
            if (target3>=0 && (Workspace::workspaces)[target3]->type == 9) {
                int cnt = 0;
                for (auto i : (Workspace::workspaces)[target3]->reserved) {
                    cnt += i.second;
                }
                if (cnt > 2) { target3=-1; }
            }
            if (target3 != -1) {
                task.push(target2);
                task.push(target3);
                (Workspace::workspaces)[target2]->reserved[(Workspace::workspaces)[target2]->type] = 1;
                //int flag = (Workspace::workspaces)[target3]->type == 8 || (Workspace::workspaces)[target3]->type == 9 ? 0 : 1;
                (Workspace::workspaces)[target3]->reserved[(Workspace::workspaces)[target2]->type] = 1;
            }
        }
    }
    else if (nine_target2 != -1)
    {
        task.push(nine_target1);
        task.push(nine_target2);
        // 添加预定
        (Workspace::workspaces)[nine_target1]->reserved[(Workspace::workspaces)[nine_target1]->type] = 1;
        (Workspace::workspaces)[nine_target2]->reserved[(Workspace::workspaces)[nine_target1]->type] = 1;
    }
    // else
    // {
    //     wait = 1;
    // }
}

double Robot::func(double holdFrame)
{ // 计算时间系数,不考虑碰撞
    return (1 - sqrt(1 - pow(1 - holdFrame / 9000, 2))) * (1 - 0.8) + 0.8;
}

void Robot::buy()
{ // 机器人的买行为

    //超过多少帧之后，不进行buy操作 TODO:


    if (task.empty())
        return; // 为空队列就不需要买

    int next_workspace = task.front();                               // 当前目标工作台
    if (is_near_workspace != -1 && load_id == 0 &&                   // 附近有工作台 且 当前未携带物品
        is_near_workspace == next_workspace &&                       // 到达的这个工作台是目标工作台
        (Workspace::workspaces)[next_workspace]->preduct_state == 1) // 到达的这个工作台已生产出来
    {
        // cerr<<id<<"do buy()..."<<endl;
        printf("buy %d\n", id);


        /* 任务出队 只能卖了才能出队 */
        task.pop();

        /* 买了之后更新 目标工作台 产品格 被预定的状态为0 */
        Workspace::workspaces[next_workspace]->reserved[Workspace::workspaces[next_workspace]->type] = 0;
        /* 买了之后更新 目标工作台 产品状态为 空 */
        Workspace::workspaces[next_workspace]->preduct_state = 0;
    }
    return;
}

void Robot::sell()
{ // 机器人的售出行为
    if (task.empty())
        return; // 为空队列就不需要卖

    int next_workspace = task.front(); // 当前任务队列的队首，返回该工作台的id
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


        // sao
        if (task.empty()) {
            double record_t = 1e9;
            if (((Workspace::workspaces)[next_workspace]->reserved[(Workspace::workspaces)[next_workspace]->type] == 0) &&
                ((Workspace::workspaces)[next_workspace]->preduct_state == 1 ||
                    (50.00 > (Workspace::workspaces)[next_workspace]->remain_preduct_time &&
                        (Workspace::workspaces)[next_workspace]->remain_preduct_time != -1))) {
                int target = -1;
                for (int targetId : (Workspace::workspaces)[next_workspace]->next_workspace) {
                    double t = get_time2(next_workspace, targetId, 0.0);
                    if (((Workspace::workspaces)[targetId]->reserved[(Workspace::workspaces)[next_workspace]->type] == 0)
                        && ((Workspace::workspaces)[targetId]->resource_state[(Workspace::workspaces)[next_workspace]->type] == 0 ||
                            ((Workspace::workspaces)[targetId]->is_full_resource() &&
                                (Workspace::workspaces)[targetId]->remain_preduct_time != -1 &&
                                (Workspace::workspaces)[targetId]->remain_preduct_time <= t &&
                                (Workspace::workspaces)[targetId]->preduct_state == 0))) {
                        // 增加判断游戏结束时候能不能卖出去，否则跳过
                        if ((double)frameID + t + 10.0 > 9000.0)
                        {
                            is_handle_collision = 0;
                            continue;
                        }
                        else {
                            is_handle_collision = 1;
                        }
                        if (t < record_t) { record_t = t; target = targetId; }
                    }
                }
                if (target >= 0 && (Workspace::workspaces)[target]->type == 9) {
                    int cnt = 0;
                    for (auto i : (Workspace::workspaces)[target]->reserved) {
                        cnt += i.second;
                    }
                    if (cnt > 2) { target = -1; }
                }
                if (target != -1) {
                    task.push(next_workspace);
                    task.push(target);
                    (Workspace::workspaces)[next_workspace]->reserved[(Workspace::workspaces)[next_workspace]->type] = 1;
                    //int flag = (Workspace::workspaces)[target3]->type == 8 || (Workspace::workspaces)[target3]->type == 9 ? 0 : 1;
                    (Workspace::workspaces)[target]->reserved[(Workspace::workspaces)[next_workspace]->type] = 1;

                }
            }
        }
    }
}

/*--------------------------------------------------------move接口0314-----------------------------------*/
void Robot::move()
{
    if (task.empty() && frameID > 8500) {
        //cerr<< "hello finish.."<<endl;
        printf("forward %d %f\n", id, 4.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }

    int target_ws_id = 0;
    if (task.empty())
    { // 需要等待
        printf("forward %d %f\n", id, 0.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }

    target_ws_id = task.front();
    /*

    if (is_near_workspace == task.front()) {
        task.pop();
        if (task.empty()) {
            printf("forward %d %f\n", id, 0.0);
            printf("rotate %d %f\n", id, 0.0);
            return;
        }
        else {
            target_ws_id = task.front();
        }
    }
    */

    if (fabs(v[0]) <= 1e-3 && fabs(v[1]) <= 1e-3)
    {
        printf("forward %d %f\n", id, 3.0);
        printf("rotate %d %f\n", id, 0.0);
        return;
    }
    // 两者相撞, 角速度基本为0
    if (x >= 1.00 && x <= 49.0 && y >= 1.00 && y <= 49.00 && is_handle_collision == 1)
    {
        for (int cid = 0; cid < 4 && cid != id; cid++)
        {
            pair<double, double> BO(robots[cid]->x - x, robots[cid]->y - y);
            double br = robots[cid]->load_id ? 0.53 : 0.45, ar = load_id ? 0.53 : 0.45;
            double BR = br + ar;

            // 求两个切点
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

            // 碰撞检测
            pair<double, double> relative_v(v[0] - robots[cid]->v[0], v[1] - robots[cid]->v[1]);
            double relative_v_m = sqrt(pow(relative_v.first, 2) + pow(relative_v.second, 2));
            double crossp1 = relative_v.first * tangency1.second - relative_v.second * tangency1.first;
            double crossp2 = relative_v.first * tangency2.second - relative_v.second * tangency2.first;
            // 进入碰撞处理
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
                        cfs = 50.0*sqrt(pow(vhope2.first, 2) + pow(vhope2.second, 2));
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

    // 速度向量和距离向量
    double targetx = (Workspace::workspaces)[target_ws_id]->x;
    double targety = (Workspace::workspaces)[target_ws_id]->y;
    pair<double, double> speed_vec(99.0 * v[0], 99.0 * v[1]), dis_vec(targetx - x, targety - y);
    // 叉乘
    double crossp = speed_vec.first * dis_vec.second - speed_vec.second * dis_vec.first;
    // 转向, <0:顺时针, >0:逆时针
    double dir = crossp < 0 ? -1 : 1;

    // 转角 cos
    double dot = speed_vec.first * dis_vec.first + speed_vec.second * dis_vec.second;
    double speed_vec_m = sqrt(speed_vec.first * speed_vec.first + speed_vec.second * speed_vec.second);
    double dis_vec_m = sqrt(dis_vec.first * dis_vec.first + dis_vec.second * dis_vec.second);
    speed_vec_m = speed_vec_m <= 0.001 ? 1.0 : speed_vec_m;
    dis_vec_m = dis_vec_m <= 0.001 ? 1.0 : dis_vec_m;
    double cos = dot / (speed_vec_m * dis_vec_m);
    cos  = cos < -1.0 ? -1.0 : cos;
    cos = cos > 1.0 ? 1.0 : cos;
    double theta = acos(cos);
    // cerr << theta << " ";

    // 转速
    double base_rs = dir * w > 0 ? 3.0 * theta : (theta + fabs(w)) * 3.0;
    double base_fs = 9.90;

    // 到达边界
    if (x <= 2.00 || x >= 48.00 || y <= 2.00 || y >= 48.00)
    {
        base_fs = 2.50;
        base_rs = dir * w > 0 ? 11.00 * theta : (theta + fabs(w)) * 11.00;
    }

    // 尤其对准目标后, 转角约为 0°时, 超快提速
    double fs = 1 / (base_rs + 0.35) * base_fs;
    double rs = base_rs * dir;

    // 靠近目标点, 减速, 准备转向, 切换目标点
    if (dis_vec_m < 1.00)
    {
        fs = 2.70 * dis_vec_m;
    }

    printf("forward %d %f\n", id, fs);
    printf("rotate %d %f\n", id, rs);
}
/*--------------------------------------------------------move接口0314-----------------------------------*/

/*----------------------------------------------------------v1.0---------------------------------------------*/
double Robot::get_time1(int target_ws_id)
{
    double targetx = (Workspace::workspaces)[target_ws_id]->x;
    double targety = (Workspace::workspaces)[target_ws_id]->y;
    // 速度向量, 方向向量
    pair<double, double> speed_vec(99.0 * v[0], 99.0 * v[1]), dis_vec(targetx - x, targety - y);

    // 夹角, 距离
    double dot = speed_vec.first * dis_vec.first + speed_vec.second * dis_vec.second;
    double speed_vec_m = sqrt(speed_vec.first * speed_vec.first + speed_vec.second * speed_vec.second);
    double dis_vec_m = sqrt(dis_vec.first * dis_vec.first + dis_vec.second * dis_vec.second);
    speed_vec_m = (speed_vec_m <= 0.001) ? 1.0 : speed_vec_m;
    dis_vec_m = (dis_vec_m <= 0.001) ? 1.0 : dis_vec_m;
    double cos = dot / (speed_vec_m * dis_vec_m);
    cos  = cos < -1.0 ? -1.0 : cos;
    cos = cos > 1.0 ? 1.0 : cos;
    double theta = acos(cos);

    // kp-角速度的P控制, bfs-base forward speed, b-fs公式常数项
    double kp = 3.0, bfs = 3.0, b = 0.35, bias = 0.3;
    // 时间公式, 单位帧, 每秒50帧
    double t1 = 50.00 * (2.0 / kp + (dis_vec_m - (bfs / (kp * theta / 3.0 + b)) * (2.0 / kp)) / 6.0 + bias);

    return t1;
}

double Robot::get_time2(int target_ws_id1, int target_ws_id2, double t1)
{
    // r2ws1从robot到第一个工作站的距离向量, ws12ws2从第一个工作站到第二个工作站的距离向量
    double x1 = (Workspace::workspaces)[target_ws_id1]->x, y1 = (Workspace::workspaces)[target_ws_id1]->y;
    double x2 = (Workspace::workspaces)[target_ws_id2]->x, y2 = (Workspace::workspaces)[target_ws_id2]->y;
    pair<double, double> r2ws1(x1 - x, y1 - y), ws12ws2(x2 - x1, y2 - y1);

    // 求模
    double r2ws1_m = sqrt(r2ws1.first * r2ws1.first + r2ws1.second * r2ws1.second);
    double ws12ws2_m = workspaces_map[target_ws_id1][target_ws_id2];
    // 以前没求过距离
    if (ws12ws2_m <= 0)
    {
        ws12ws2_m = sqrt(ws12ws2.first * ws12ws2.first + ws12ws2.second * ws12ws2.second);
        workspaces_map[target_ws_id1][target_ws_id2] = workspaces_map[target_ws_id2][target_ws_id1] = ws12ws2_m;
    }
    double dot = r2ws1.first * ws12ws2.first + r2ws1.second * ws12ws2.second;
    r2ws1_m = r2ws1_m <= 0.001 ? 1.0 : r2ws1_m;
    ws12ws2_m = ws12ws2_m <= 0.001 ? 1.0 : ws12ws2_m;
    double cos = dot / (r2ws1_m * ws12ws2_m);
    cos  = cos < -1.0 ? -1.0 : cos;
    cos = cos > 1.0 ? 1.0 : cos;
    double theta = acos(cos);

    // kp-角速度的P控制, bfs-base forward speed, b-fs公式常数项
    double kp = 3.0, bfs = 3.0, b = 0.35, bias = 0.3;
    // 时间公式, 单位帧, 每秒50帧
    double t2 = 50.00 * (2.0 / kp + (ws12ws2_m - (bfs / (kp * theta / 3.0 + b)) * (2.0 / kp)) / 6.0 + bias);

    return t2;
}
double Robot::get_time3(int target_ws_id1, int target_ws_id2, int target_ws_id3, double t2)
{
    // r2ws1从robot到第一个工作站的距离向量, ws12ws2从第一个工作站到第二个工作站的距离向量
    double x1 = (Workspace::workspaces)[target_ws_id1]->x, y1 = (Workspace::workspaces)[target_ws_id1]->y;
    double x2 = (Workspace::workspaces)[target_ws_id2]->x, y2 = (Workspace::workspaces)[target_ws_id2]->y;
    double x3 = (Workspace::workspaces)[target_ws_id3]->x, y3 = (Workspace::workspaces)[target_ws_id3]->y;
    pair<double, double> ws12ws2(x2 - x1, y2 - y1), ws22ws3(x3 - x2, y3 - y2);

    // 求模
    double ws12ws2_m = workspaces_map[target_ws_id1][target_ws_id2];
    double ws22ws3_m = workspaces_map[target_ws_id2][target_ws_id3];
    // 以前没求过距离
    if (ws12ws2_m <= 0) {
        ws12ws2_m = sqrt(ws12ws2.first * ws12ws2.first + ws12ws2.second * ws12ws2.second);
        workspaces_map[target_ws_id1][target_ws_id2] = workspaces_map[target_ws_id2][target_ws_id1] = ws12ws2_m;
    }
    if (ws22ws3_m <= 0) {
        ws22ws3_m = sqrt(ws22ws3.first * ws22ws3.first + ws22ws3.second * ws22ws3.second);
        workspaces_map[target_ws_id2][target_ws_id3] = workspaces_map[target_ws_id3][target_ws_id2] = ws22ws3_m;
    }
    double dot = ws12ws2.first * ws22ws3.first + ws12ws2.second * ws22ws3.second;
    ws22ws3_m = ws22ws3_m <= 0.001 ? 1.0 : ws22ws3_m;
    ws12ws2_m = ws12ws2_m <= 0.001 ? 1.0 : ws12ws2_m;
    double cos = dot / (ws22ws3_m * ws12ws2_m);
    cos = cos < -1.0 ? -1.0 : cos;
    cos = cos > 1.0 ? 1.0 : cos;
    double theta = acos(cos);

    // kp-角速度的P控制, bfs-base forward speed, b-fs公式常数项
    double kp = 3.0, bfs = 3.0, b = 0.35, bias = 0.3;
    // 时间公式, 单位帧, 每秒50帧
    double t3 = 50.00 * (2.0 / kp + (ws22ws3_m - (bfs / (kp * theta / 3.0 + b)) * (2.0 / kp)) / 6.0 + bias);

    return t3;
}
/*----------------------------------------------------------v1.0---------------------------------------------*/

/* 尝试使用distance替代时间直接计算 */

// double Robot::get_time1_by_dis(int target_ws_id){
//     return workspaces_map[id][target_ws_id];
// }

// double Robot::get_time2_by_dis(int target_ws_id1, int target_ws_id2, double t1);