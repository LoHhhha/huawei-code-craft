#pragma once

#include<bits/stdc++.h>
using namespace std;

using ll = long long;
using pii = pair<int, int>;

// #define int ll
// #define endl "\n"	// 是否保留待定
#define OPTIO std::ios::sync_with_stdio(0), std::cin.tie(0), std::cout.tie(0)
#define LLONG_INF 0x3f3f3f3f3f3f3f3f
#define INT_INF 0x3f3f3f3f
#define GRAPH_SIZE 200	        // 地图大小
#define ROBOT_NUM 	10	        // 机器人数量
#define BERTH_NUM 	10	        // 泊位数量
#define BOAT_NUM 	5	        // 船数量
#define FRAME_COUNT 15000       // 模拟总帧数
#define PACKET_TIME_OUT 1000    // 货物过期
#define RIGHT 		0	        // 机器人方向：右
#define LEFT 		1	        // 机器人方向：左
#define UP 			2	        // 机器人方向：上
#define DOWN 		3	        // 机器人方向：下
#define BERTH_BIT 	1	        // 泊位标记
#define ROBOT_BIT 	2	        // 机器人标记
#define PACKET_BIT 	4	        // 货物标记


int dir[4][2] = {{0,1}, {0,-1}, {-1,0}, {1,0}};		// 移动方向，下标与机器人指令方向一致

vector<vector<int>> graph(GRAPH_SIZE, vector<int>(GRAPH_SIZE));	// 地图 vector 障碍:-1 空地:0 停泊点:1 机器人:2 货物:4 （二进制）

vector<vector<map<int,int>>> book(GRAPH_SIZE, vector<map<int,int>>(GRAPH_SIZE));	// pii:{book_frame,id} 点被预定的情况

int frame;	// 当前帧数

int money;	// 当前金钱数

bool robot_can_go[GRAPH_SIZE][GRAPH_SIZE]{0};  // 维护机器人能到达的点

pii go_to_which_berth[GRAPH_SIZE][GRAPH_SIZE];		// 场上每一个点去哪一个泊位{id, dict} 注意：当id==-1或者dict==INT_INF时不可达！
bool use_berth[BERTH_NUM]{0};		// 泊位是否被使用
