#include "main.h"
#include "robot.h"
#include "workspace.h"
#include "a_star.h"
#include "radar.h"
#include "init.h"

#include <iostream>
#include <vector>
#include <cstring>
#include <sstream> // istringstream
#include <math.h>
#include <cstring>

using namespace std;

/****************参数*******************/
int avoid_range =  5; // 5  图2：2
int avoid_index_differ =  2; // 2  图2：1
int point_near_value = 3;
double wait_product = 0.0 ;//等待生产时间

/* 全局记录是否存在9号工作台 */
bool nine_workspace_is = 0;

bool optimize2 = 0;

/* 红蓝方，蓝色为b，红色为r */
char color;

/* 全局记录7号工作台数量 */
int num_of_seven = 0;
int num_of_4 = 0;
int num_of_5 = 0;
int num_of_6 = 0;

/* 全局帧交互id */
int frameID;

/* 工作台数量 */
int nums_of_workspaces = 0;
int nums_of_enemy_workspaces = 0;

/* 存储各个工作台i 到 工作台j 之间的距离 */
double workspaces_map[N][N];

/* 存储各个工作台i 到 工作台j 之间的权重 */
double workspaces_map_weight[N][N];

/* 记录是哪张图 */
int mapID = 0;

/* 存储各 workspace[i][j] 类型之间的卖关系 */
double workspace_sell_ok[10][10] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

char line[20480];

vector<string> my_split(string s)
{ // 分割字符串函数，用于处理输入数据
    vector<string> vec;
    int sublen = 0;
    int size = s.size();
    for (int i = 0; i < size; i++)
    {
        if (s[i] != ' ')
        {
            sublen++;
            continue;
        }
        vec.push_back(s.substr(i - sublen, sublen));
        sublen = 0;
    }
    if (sublen != 0)
    {
        vec.push_back(s.substr(size - sublen, sublen));
    }
    return vec;
}

void init(string line, int count)
{
    for (int i = 0; i < line.size(); i++)
    {
        if (line[i] == 'A')
        { // 机器人初始化点
            if(color == 'b') // blue为A
                (Robot::robots).push_back(new Robot(i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, 'b'));
            else 
                (Robot::enemy_robots).push_back(new Robot(i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, 'r'));
        }
        else if(line[i] == 'B')
        {
            if(color == 'r') // red为B
                (Robot::robots).push_back(new Robot(i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, 'r'));
            else 
                (Robot::enemy_robots).push_back(new Robot(i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, 'b'));
        }
        else if (line[i] <= '9' && line[i] >= '1')
        { // 蓝方工作台初始化点
            if(color == 'b')
            {
                (Workspace::workspaces).push_back(new Workspace((line[i]-'0'), i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, count, i, 'b'));
                nums_of_workspaces++;
            }
            else
            {
                (Workspace::enemy_workspaces).push_back(new Workspace((line[i]-'0'), i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, count, i, 'r'));
                nums_of_enemy_workspaces++;
            }
            
        }
        else if (line[i] <= 'i' && line[i] >= 'a')
        {   // 红方工作台初始化点
            if(color == 'r')
            {
                (Workspace::workspaces).push_back(new Workspace((line[i]-'a'+1), i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, count, i, 'r'));
                nums_of_workspaces++;
            }
            else
            {
                (Workspace::enemy_workspaces).push_back(new Workspace((line[i]-'a'+1), i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, count, i, 'b'));
                nums_of_enemy_workspaces++;
            }
        }

        if (line[i] == '9') nine_workspace_is = 1;
        if(line[i] == '7'){
            num_of_seven ++;
        }
        if(line[i] == '4'){
            num_of_4 ++;
        }
        if(line[i] == '5'){
            num_of_5 ++;
        }
        if(line[i] == '6'){
            num_of_6 ++;
        }
    }
    // 赋值graph
    for(int i = 0; i < width; i++){
        if(i>=width){
            cerr<<"boundary error.."<<endl;
            continue;
        }
        graph[count][i] = line[i];
    }
}

bool readUntilOK()
{
    fgets(line, sizeof line, stdin);
    if(line[0] == 'B' && line[1] == 'L' && line[2] == 'U' && line[3] == 'E') color = 'b';
    else if(line[0] == 'R' && line[1] == 'E' && line[2] == 'D') color = 'r';

    int count = 0; // 表示第几行line
    while (fgets(line, sizeof line, stdin))
    {
        if (line[0] == 'O' && line[1] == 'K')
        {
            return true;
        }
        init(line, count);
        count++;
    }
    return false;
}

void readEveryFrameUntilOK()
{
	vector<string> vec;
    // 金额
    fgets(line, sizeof line, stdin);
    vec = my_split(line);
    Robot::money = atoi(vec[1].c_str());
    // 工作台
    fgets(line, sizeof line, stdin);
    vec = my_split(line);
    nums_of_workspaces = atoi(vec[0].c_str());
    for(int i = 0; i < nums_of_workspaces; ++i)
    {
    	fgets(line, sizeof line, stdin);
    	vec = my_split(line);
        (Workspace::workspaces)[i]->id = i;
        (Workspace::workspaces)[i]->init_frame(vec);
	}
    // 机器人
    for(int i = 0; i < 4; ++i)
    {
    	fgets(line, sizeof line, stdin);
    	vec = my_split(line);
    	(Robot::robots)[i]->id = i;
    	(Robot::robots)[i]->init_frame(vec);
	}
    // 雷达
    for(int i = 0; i < 4; ++i)
    {
    	fgets(line, sizeof line, stdin);
    	vec = my_split(line);
        for(int j = 0;j < 360; j++)
        {
            (Robot::robots)[i]->radar[j] = stod(vec[j]);
        }
        // cerr<<"radr ok"<<endl;
	}
    // OK
    fgets(line, sizeof line, stdin);
}

int move_pcnt = 0; // pmove的测试变量

int main()
{
    srand(time(0));
    readUntilOK();

    /* 给各工作站编号 */
    int k = 0;
    for (Workspace *ws : (Workspace::workspaces))
    {
        ws->id = k;
        k++;
        Workspace::workspaces_type_mp[ws->type].push_back(ws);
        ws->in_danger_corner();
    }
    k = 0; // 敌方工作台为负id
    for (Workspace *ws : (Workspace::enemy_workspaces))
    {
        ws->id = k;
        k++;
        Workspace::enemy_workspaces_type_mp[ws->type].push_back(ws);
        ws->in_danger_corner();
    }
    k = 0;
    for(Robot* rot : Robot::robots){
        rot->id = k;
        k++;
    }

    // /* 初始化工作台之间的距离 */
    // for (auto wk_sour : (Workspace::workspaces))
    // {
    //     for (auto wk_des : (Workspace::workspaces))
    //     {
    //         if (wk_sour->id == wk_des->id) // 同一个节点
    //             workspaces_map[wk_sour->id][wk_des->id] = 0;
    //         else if (!workspace_sell_ok[wk_sour->type][wk_des->type]) // 节点不可达
    //             workspaces_map[wk_sour->id][wk_des->id] = -1;
    //         else // 计算欧氏距离
    //             workspaces_map[wk_sour->id][wk_des->id] =
    //                 sqrt(pow(wk_sour->x - wk_des->x, 2) + pow(wk_sour->y - wk_des->y, 2));
    //     }
    // }

    // 不存在7号工作台，需要更改456 可以 到9号工作台的售卖 
    if(!num_of_seven){
        for(int i = 4;i <= 6; i++) workspace_sell_ok[i][9] = 1; 
    }
    
    // 初始化栅格图中所有路径，初始化工作台和机器人 可达的工作台
    init_allpaths();
    // cerr<<"finish init...a*"<<endl;
    init_enemy_allpaths();
    // cerr<<"finish init enemy...a*"<<endl;
    init_to_enemy_allpaths();
    // cerr<<"finish init to_enemy_allpaths...a*"<<endl;

    // 获得地图难易程度比较
    int easy = easy_score();
    // cerr<< "easy is"<<easy<<endl;

    /* 判断地图类型 */
    mapID = 1; // 默认为图1
    for(auto rob : Robot::robots) 
    {
        if(rob->reachable_wk_id_v.size() == 0) mapID = 2; // 存在机器人在不连通的敌方
    }
    if(easy != 0) mapID = 3; // 难易不同的是图3
    if(nums_of_workspaces == 0 || nums_of_enemy_workspaces == 0) mapID = 4; // 4 号图一方无工作台
    cerr<< "mapID: "<< mapID <<endl; 

/********************************根据地图调整参数**************************************************/
    //TODO:
    if(mapID == 3){
        if(color == 'r'){
            avoid_range = 3;// 2
            avoid_index_differ = 1;// 1
            point_near_value = 2;
        }
    }


    // 初始化各个工作台可达的 next_workspace 
    for (auto wk_sour : (Workspace::workspaces))
        wk_sour->init_next_workspace();

    // 初始化游走进攻工作台集群
    // all_enemy_cluster[0].cluster是最推荐进行游走进攻的集群
    init_all_enemy_cluster();
    
    init_all_hot_stand_space();
    // cerr<<"init hss success..."<<endl;

    Robot::stand_attack(); // 一开始占据完全

    puts("OK");
    fflush(stdout);
    // (Robot::robots)[0]->next_workspace = move_pcnt;
    while (scanf("%d", &frameID) != EOF)
    {
        readEveryFrameUntilOK();
        printf("%d\n", frameID);
        radar_update_enermy_robot();
        /*cerr << Robot::enemy_robots.size() << endl;
        for (Robot* rot : Robot::enemy_robots) {
            cerr << "("<<rot->x << " " << rot->y<<")" << " ";
        }
        cerr << endl;*/
        for (int robotId = 0; robotId < 4; robotId++)//调试
        {
            (Robot::robots)[robotId]->stand_space_attack();//每帧通过avoid_times检测是否需要进攻
            (Robot::robots)[robotId]->simple_seek();
            (Robot::robots)[robotId]->sell();
            (Robot::robots)[robotId]->buy();
            (Robot::robots)[robotId]->friendly_conflict_detect();
            if(color == 'r'){
                (Robot::robots)[robotId]->enemy_conflict_detect();
            }
            (Robot::robots)[robotId]->conflict_avoidance();
            Robot::robots[robotId]->continue_attack();
            Robot::robots[robotId]->detect_to_reseek(); // 防御，敌方干自己时候重新reseek
            (Robot::robots)[robotId]->move_stable();
            if(frameID == 12000){
                cerr<<"robot id="<<robotId<<" + load_id="<<(Robot::robots)[robotId]->load_id<<endl;
            }
        }
        /*--------------------------------------pmove测试-------------------------------*/
        // if (fabs((Workspace::workspaces)[move_pcnt]->x - (Robot::robots)[0]->x) < 0.4 && fabs((Workspace::workspaces)[move_pcnt]->y - (Robot::robots)[0]->y) < 0.4)
        //{
        //      move_pcnt++;
        //      (Robot::robots)[0]->next_workspace = move_pcnt;
        // }
        // (Robot::robots)[0]->baseline_move((Workspace::workspaces));

        /*--------------------------------------pmove测试------------------------------*/

        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}
