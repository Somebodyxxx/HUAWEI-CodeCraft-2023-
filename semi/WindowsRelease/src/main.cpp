#include "main.h"
#include "robot.h"
#include "workspace.h"
#include "a_star.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream> // istringstream
#include <math.h>

using namespace std;


/* 全局记录是否存在9号工作台 */
bool nine_workspace_is = 0;

/********************************************************************/
//TODO:可调
//关闭合成7的策略
int close_do7_frame = 13760;
double reach_extern_time = 10.0;
double end_extern_time = 200.0;
double total_average_speed = 5.0;
int avoid_range = 10;
int avoid_conflict_gap = 3; // i - j > avoid_conflict_gap

/*********************************************************************/
bool optimize2 = 0;

/* 全局记录7号工作台数量 */
int num_of_4 = 0;
int num_of_5 = 0;
int num_of_6 = 0;

/* 全局记录7号工作台数量 */
int num_of_seven = 0;

/* 全局帧交互id */
int frameID;

/* 工作台数量 */
int nums_of_workspaces = 0;

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
    if (line.find("..") != -1 || line[0] == '.' || line[1] == '.') // 处理初始地图数据，初始化robots workspaces位置
    {
        for (int i = 0; i < line.size(); i++)
        {
            if (line[i] == 'A')
            { // 机器人初始化点
                (Robot::robots).push_back(new Robot(i * 0.5 + 0.25, 50 - count * 0.5 - 0.25));
            }
            if (line[i] <= '9' && line[i] >= '1')
            { // 工作台初始化点
                (Workspace::workspaces).push_back(new Workspace((line[i]-'0'), i * 0.5 + 0.25, 50 - count * 0.5 - 0.25, count, i));
                nums_of_workspaces++;
            }
            if (line[i] == '9') nine_workspace_is = 1;
            if(line[i] == '7'){
                num_of_seven ++;
            }
            if (line[i] == '4') {
                num_of_4++;
            }
            if (line[i] == '5') {
                num_of_5++;
            }
            if (line[i] == '6') {
                num_of_6++;
            }
        }
        for(int i = 0; i < width; i++){
            if(i>=width){
                cerr<<"boundary error.."<<endl;
                continue;
            }
            graph[count][i] = line[i];
        }
    }
    else
    {
        // cerr << line << endl;
        vector<string> vec = my_split(line);
        if (line[0] == ' ' && vec.size() == 1)
        {
            frameID = atoi(vec[0].c_str());
            Robot::money = atoi(vec[1].c_str());
        }
        if (vec.size() == 1)
        {
            nums_of_workspaces = atoi(vec[0].c_str());
        }
        if (vec.size() == 6)
        {
            (Workspace::workspaces)[count - 2]->id = count - 2;
            (Workspace::workspaces)[count - 2]->init_frame(vec);
        }
        if (vec.size() == 10)
        {
            (Robot::robots)[count - 2 - nums_of_workspaces]->id = count - 2 - nums_of_workspaces;
            (Robot::robots)[count - 2 - nums_of_workspaces]->init_frame(vec);
        }
    }
}

bool readUntilOK()
{
    char line[1024];
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

int move_pcnt = 0; // pmove的测试变量

int main()
{
    srand(time(0));
    readUntilOK();

    // /* 判断地图类型 */
    if(Workspace::workspaces[0]->type == 2 && 
    Workspace::workspaces[1]->type == 1 &&
    Workspace::workspaces[2]->type == 7 &&
    Workspace::workspaces[3]->type == 7) mapID = 1;
    else if(Workspace::workspaces[0]->type == 8 &&
    Workspace::workspaces[1]->type == 1 &&
    Workspace::workspaces[2]->type == 8 &&
    Workspace::workspaces[3]->type == 8) mapID = 2;
    // else if(Workspace::workspaces[0]->type == 3) mapID = 3;
    // else if(Workspace::workspaces[1]->type == 7) mapID = 4;
    if(mapID == 1){//TODO:
        total_average_speed = 5.5;
        end_extern_time = 0.0;
        reach_extern_time = 0.0;
    }
    if(mapID == 2){
        total_average_speed = 4.5;
        end_extern_time = 50.0;
    }


    /* 给各工作站编号 */
    int k = 0;
    for (Workspace *ws : (Workspace::workspaces))
    {
        ws->id = k;
        k++;
    }

    // /* 判断地图类型 */
    // if(Workspace::workspaces[0]->type == 1) mapID = 1;
    // else if(Workspace::workspaces[0]->type == 6) mapID = 2;
    // else if(Workspace::workspaces[0]->type == 3) mapID = 3;
    // else if(Workspace::workspaces[0]->type == 7) mapID = 4;

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
    // if(!num_of_seven){
    //     for(int i = 4;i <= 6; i++) workspace_sell_ok[i][9] = 1; 
    // }
    
    // 初始化栅格图中所有路径，初始化工作台和机器人 可达的工作台
    init_allpaths();
    // cerr<<"finish init...a*"<<endl;
    
    // 初始化各个工作台可达的 next_workspace 
    for (auto wk_sour : (Workspace::workspaces))
        wk_sour->init_next_workspace();

    puts("OK");
    fflush(stdout);

    // (Robot::robots)[0]->next_workspace = move_pcnt;
    while (scanf("%d", &frameID) != EOF)
    {
        readUntilOK();
        printf("%d\n", frameID);
        for (int robotId = 0; robotId < 4; robotId++)//调试
        {
            (Robot::robots)[robotId]->simple_seek();
            (Robot::robots)[robotId]->sell();
            (Robot::robots)[robotId]->buy();
            (Robot::robots)[robotId]->conflict_avoidance();
            if(mapID == 2){
               (Robot::robots)[robotId]->move_stable(); 
            }else{
                (Robot::robots)[robotId]->move_stable3(); 
            }
            
            // (Robot::robots)[robotId]->move_test();

            if(frameID == 15000){
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
