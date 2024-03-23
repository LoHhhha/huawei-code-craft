#include "Param.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"


int dir[4][2] = {{0,1}, {0,-1}, {-1,0}, {1,0}};


int frame=0;
int money=0;

int trans_packet_count=0;


int graph[GRAPH_SIZE][GRAPH_SIZE]{0};
set<int> book[GRAPH_SIZE][GRAPH_SIZE];
// vector<vector<int>> graph(GRAPH_SIZE, vector<int>(GRAPH_SIZE));	// 地图 vector 障碍:-1 空地:0 停泊点:1 机器人:2 货物:4 （二进制）
// vector<vector<map<int,int>>> book(GRAPH_SIZE, vector<map<int,int>>(GRAPH_SIZE));	// pii:{book_frame,id} 点被预定的情况
map<int,array<array<pii,GRAPH_SIZE>,GRAPH_SIZE>>go_to_which_berth;;		// 在泊位为mask时场上每一个点去哪一个泊位{id, dict} 注意：当id==-1或者dict==INT_INF时不可达！
bool robot_can_go[GRAPH_SIZE][GRAPH_SIZE]{0};  // 维护机器人能到达的点
array<array<array<bool,GRAPH_SIZE>,GRAPH_SIZE>,(1<<BERTH_NUM)>use_berth_can_go;  // 维护选择的机器人能到达的点
unordered_set<int> berth_point_hash;
vector<int>berth_block_order[BERTH_NUM];
int current_berth_use_hash=0;


bool use_berth[BERTH_NUM]{0};		// 泊位是否被使用
vector<Berth> berth(BERTH_NUM);		// 码头 vector


vector<Boat> boat(BOAT_NUM);


MsgHandler msg_handler;


int packet_id = 0;					// 当前最后一个货物id (第一个货物id为1) 
map<int, Packet> packet;			// 货物id -> 货物信息
map<int,int> hash2packet;           // point_hash(x*GRAPH_SIZE+y) 转化为货物id
set<int> unbooked_packet;	        // 未被预定的货物 id


vector<Robot> robot(ROBOT_NUM);