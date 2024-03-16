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

struct Robot;		// 机器人结构体
struct Packet;		// 货物结构体
struct Boat;		// 船结构体
struct Berth;		// 泊位结构体
struct MsgHandler;

int dir[4][2] = {{0,1}, {0,-1}, {-1,0}, {1,0}};		// 移动方向，下标与机器人指令方向一致


int frame;	// 当前帧数
int money;	// 当前金钱数

// ---------- begin graph ----------
vector<vector<int>> graph(GRAPH_SIZE, vector<int>(GRAPH_SIZE));	// 地图 vector 障碍:-1 空地:0 停泊点:1 机器人:2 货物:4 （二进制）
vector<vector<map<int,int>>> book(GRAPH_SIZE, vector<map<int,int>>(GRAPH_SIZE));	// pii:{book_frame,id} 点被预定的情况
pii go_to_which_berth[GRAPH_SIZE][GRAPH_SIZE];		// 场上每一个点去哪一个泊位{id, dict} 注意：当id==-1或者dict==INT_INF时不可达！
bool robot_can_go[GRAPH_SIZE][GRAPH_SIZE]{0};  // 维护机器人能到达的点
// ---------- end graph ----------


// ---------- begin packet ----------
int packet_id = 0;					// 当前最后一个货物id (第一个货物id为1) 
map<int, Packet> packet;			// 货物id -> 货物信息
map<int,int> hash2packet;           // point_hash(x*GRAPH_SIZE+y) 转化为货物id
set<int> unbooked_packet;	        // 未被预定的货物 id
// ---------- end packet ----------


// ---------- begin robot ----------
vector<Robot> robot(ROBOT_NUM);	// 机器人 vector
// ---------- end robot ----------


// ---------- begin boat ----------
vector<Boat> boat(BOAT_NUM);		// 船 vector
// ---------- end boat ----------


// ---------- begin berth ----------
bool use_berth[BERTH_NUM]{0};		// 泊位是否被使用
vector<Berth> berth(BERTH_NUM);	// 码头 vector
// ---------- end berth ----------


// ---------- begin msgHandler ----------
MsgHandler msg_handler;
// ---------- end msgHandler ----------