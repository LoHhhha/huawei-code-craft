#pragma once

#include<bits/stdc++.h>


// #define DE_BUG		// 调试模式
#define DEBUG_STATE 1	// 调试模式： 0：关闭，1：终端，2：cph
#define DEBUG_FRAME 5	// 调试模式：调试帧数

#define ENABLE_STDERR   // 开启stderr输出

#ifdef DE_BUG	
	#define FRAME_TO_RUN DEBUG_FRAME	
	#define THIS_DEBUG_STATE DEBUG_STATE
#else
	#define FRAME_TO_RUN FRAME_COUNT
	#define THIS_DEBUG_STATE 0
#endif

#define OUTPUT_DEBUG_INFO	// 输出调试信息
#ifdef OUTPUT_DEBUG_INFO
	#include "DEBUG.h"
    using namespace DEBUG_;
#endif



using namespace std;
using ll = long long;
using pii = pair<int, int>;


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
#define endl '\n'


struct Boat;
struct Berth;
struct Packet;
struct Robot;
class MsgHandler;


extern int dir[4][2];   // 移动方向，下标与机器人指令方向一致


extern int frame;	// 当前帧数
extern int money;	// 当前金钱数

// ---------- begin graph ----------
extern vector<vector<int>> graph;	// 地图 vector 障碍:-1 空地:0 停泊点:1 机器人:2 货物:4 （二进制）
extern vector<vector<map<int,int>>> book;	// pii:{book_frame,id} 点被预定的情况
extern pii go_to_which_berth[GRAPH_SIZE][GRAPH_SIZE];		// 场上每一个点去哪一个泊位{id, dict} 注意：当id==-1或者dict==INT_INF时不可达！
extern bool robot_can_go[GRAPH_SIZE][GRAPH_SIZE];  // 维护机器人能到达的点
// ---------- end graph ----------


// ---------- begin berth ----------
extern bool use_berth[BERTH_NUM];		// 泊位是否被使用
extern vector<Berth> berth;	// 码头 vector
// ---------- end berth ----------


// ---------- begin boat ----------
extern vector<Boat> boat;	// 船 vector
// ---------- end boat ----------


// ---------- begin message ----------
extern MsgHandler msg_handler;
// ---------- end message ----------


// ---------- begin packet ----------
extern int packet_id;						// 当前最后一个货物id (第一个货物id为1) 
extern map<int, Packet> packet;				// 货物id -> 货物信息
extern map<int,int> hash2packet;           	// point_hash(x*GRAPH_SIZE+y) 转化为货物id
extern set<int> unbooked_packet;	        // 未被预定的货物 id
// ---------- end packet ----------


// ---------- begin robot ----------
extern vector<Robot> robot;	// 机器人 vector
// ---------- end robot ----------