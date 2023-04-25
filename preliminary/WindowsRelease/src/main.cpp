#include "main.h"
#include "robot.h"
#include "workspace.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream> // istringstream
#include <math.h>

using namespace std;

/* 全局记录是否存在9号工作台 */
bool nine_workspace_is = 0;

/* 全局记录是否存在7号工作台 */
bool seven_workspace_is = 0;

int num_of_seven = 0;

/* 全局帧交互id */
int frameID;

/* 工作台数量 */
int nums_of_workspaces;

/* 存储各个工作台i 到 工作台j 之间的距离 */
double workspaces_map[N][N];

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
    // istringstream input(s);
    // vector<string> vec;
    // string temp;
    // while ( input >> temp ){
    //     vec.push_back(temp);
    //     //cerr<<temp;
    // }
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
                (Workspace::workspaces).push_back(new Workspace((line[i]-'0'), i * 0.5 + 0.25, 50 - count * 0.5 - 0.25));
            }
            if(line[i] == '7'){
                num_of_seven ++;
            }
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
            // int k = 0;
            // for(Workspace* ws : (Workspace::workspaces)){
            //     ws->id = k;
            //     k++;
            (Workspace::workspaces)[count - 2]->id = count - 2;
            // cerr<<"workspace init id:"<<count-2<<endl;

            // for(string s: vec){
            //     cerr<<s<<"+";
            // }
            // cerr<<endl;
            (Workspace::workspaces)[count - 2]->init_frame(vec);
            // cerr<<"have done wk init_frame();"<<endl;
            // cerr<<(Workspace::workspaces)[count -2]->preduct_state<<endl;

            // if((Workspace::workspaces)[count - 2]->preduct_state ==0){//如果某工作台无现成产品，工作台变为不可达，机器人需调换目标
            //     for(Robot* rot : (Robot::robots)){
            //         if (rot->next_workspace == (Workspace::workspaces)[count - 2]->id)
            //         {
            //             rot->next_workspace = -1;
            //         }
            //     }
            // }
        }
        if (vec.size() == 10)
        {
            // int k = 0;
            // for(Robot* rot : (Robot::robots)){
            //     rot->id = k;
            //     k++;
            //     rot->init_frame(vec);
            // }
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

    /* 给各工作站编号 */
    int k = 0;
    for (Workspace *ws : (Workspace::workspaces))
    {
        ws->id = k;
        k++;
    }

    /* 初始化工作台之间的距离 */
    for (auto wk_sour : (Workspace::workspaces))
    {
        for (auto wk_des : (Workspace::workspaces))
        {
            if (wk_sour->id == wk_des->id) // 同一个节点
                workspaces_map[wk_sour->id][wk_des->id] = 0;
            else if (!workspace_sell_ok[wk_sour->type][wk_des->type]) // 节点不可达
                workspaces_map[wk_sour->id][wk_des->id] = -1;
            else // 计算欧氏距离
                workspaces_map[wk_sour->id][wk_des->id] =
                    sqrt(pow(wk_sour->x - wk_des->x, 2) + pow(wk_sour->y - wk_des->y, 2));
        }

        // 记录是否有9号工作台
        if (wk_sour->type == 9) nine_workspace_is = 1;

        // 记录是否存在7号工作台，不存在需要更改456 可以 到9号工作台的售卖
        if(wk_sour->type == 7) seven_workspace_is = 1;
    }

    // 不存在7号工作台，需要更改456 可以 到9号工作台的售卖 
    // TODO: 改了之后456都送去9, 应该是价值问题导致不送9更优
    if(!seven_workspace_is){
        for(int i = 4;i <= 6; i++) workspace_sell_ok[i][9] = 1; 
    }

    // 初始化各个工作台可达的next_workspace 
    for (auto wk_sour : (Workspace::workspaces))
        wk_sour->init_next_workspace();

    puts("OK");
    fflush(stdout);
    // (Robot::robots)[0]->next_workspace = move_pcnt;
    while (scanf("%d", &frameID) != EOF)
    {
        readUntilOK();
        printf("%d\n", frameID);
        for (int robotId = 0; robotId < 4; robotId++)
        {
            (Robot::robots)[robotId]->seek();
            (Robot::robots)[robotId]->sell();
            (Robot::robots)[robotId]->buy();
            (Robot::robots)[robotId]->move();
            if(frameID == 9000){
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
