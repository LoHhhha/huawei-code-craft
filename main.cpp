#include<bits/stdc++.h>
#include"things.h"
using namespace std;

using ll = long long;
using pii = pair<int, int>;

// #define int ll
// #define endl "\n"	// 是否保留待定
#define OPTIO std::ios::sync_with_stdio(0), std::cin.tie(0), std::cout.tie(0)
#define LLONG_INF 0x3f3f3f3f3f3f3f3f
#define INT_INF 0x3f3f3f3f
#define GRAPH_SIZE 200
#define ROBOT_NUM 10
#define BERTH_NUM 10
#define BOAT_NUM 5

// #define DE_BUG
#ifdef DE_BUG
    #include "DEBUG.h"
    using namespace DEBUG_;
#endif

#define RIGHT 	0
#define LEFT 	1
#define UP 		2
#define DOWN 	3


vector<Robot> robot(ROBOT_NUM);	// 机器人 vector

vector<Berth> berth(BERTH_NUM);	// 码头 vector

int packet_id = 0;			// 当前最后一个货物id
map<int, Packet> packet;	// 货物

vector<vector<int>> graph(GRAPH_SIZE, vector<int>(GRAPH_SIZE));	// 地图 vector

vector<Boat> boat(BOAT_NUM);	// 船 vector

vector<vector<set<int>>> book(GRAPH_SIZE, vector<set<int>>(GRAPH_SIZE));	// 点被预定的情况

int frame;	// 当前帧数

int money;	// 当前金钱数

// 初始化
void init() {
	// 地图初始化
	auto f = [](char c) {
		if (c == '#' || c == '*') return -1;	// 障碍或海洋
		else if (c == '.') return 0;	// 空地
		else if (c == 'B') return 1;	// 泊位
		else if (c == 'A') return 3;	// 机器人
		else return -100;				// 输入有误
	};
	for (int i=0;i<GRAPH_SIZE;i++) {
		for (int j=0;j<GRAPH_SIZE;j++) {
			graph[i][j] = f(cin.get());
		}
		cin.get();
	}

	// 泊位初始化
	for (int i=0;i<BERTH_NUM;i++) {
		int id;
		cin >> id >> berth[i].x >> berth[i].y >> berth[i].transport_time >> berth[i].loading_speed;
	}

	int boat_capacity;	// 船的容量
	cin >> boat_capacity;	
	for(int i=0;i<BOAT_NUM;i++) {
		boat[i].load = 0;
		boat[i].status = 2;
		boat[i].berth_id = -1;
		boat[i].capacity = boat_capacity;
	}
	string okk;
	cin >> okk;

	// 在这里写初始化代码

	cout << "OK" << endl;
}

// 获取帧输入
void get_input() {
	cin >> frame >> money;	// 获取帧数和金钱数

	// 货物
	int goods_num;
	cin >> goods_num;
	for (int i=0;i<goods_num;i++) {
		int x, y, packet_money;
		cin >> x >> y >> packet_money;
		Packet p(++packet_id, x, y, packet_money, frame + 1000);	// 在 1000 帧后过期
		packet[packet_id] = p;
	}

	// 机器人
	for (int i=0;i<ROBOT_NUM;i++) {
		int have_packet, x, y, robot_status;	// have_packet表示机器人是否有货物，Robot内packet_id表示货物id
		cin >> have_packet >> x >> y >> robot_status;
		// todo：packet_id待处理

		graph[robot[i].x][robot[i].y] = 0;
		graph[x][y] = 3;

		robot[i].x = x;
		robot[i].y = y;
		robot[i].status = robot_status;
	}

	// 船      
	for (int i=0;i<BOAT_NUM;i++) {
		int boat_status, boat_target;
		cin >> boat_status >> boat_target;
		boat[i].status = boat_status;
		boat[i].berth_id = boat_target;
	}

	string okk;
	cin >> okk;
}


int tmp_dict[GRAPH_SIZE][GRAPH_SIZE];
pair<int,queue<int>> get_a_path(int robot_id,int x,int y){
	for(int i=0;i<GRAPH_SIZE;i++){
		for(int j=0;j<GRAPH_SIZE;j++){
			tmp_dict[i][j]=INT_INF;
		}
	}
	
	Robot &current_robot=robot[robot_id];
	int robot_current_x=current_robot.x,robot_current_y=current_robot.y;
	tmp_dict[robot_current_x][robot_current_y]=frame;

	// point_hash, dict
	auto cmp=[&](auto &p1,auto &p2){
		return p1.second>p2.second;
	};
	priority_queue<pair<int,int>,vector<pair<int,int>>,decltype(cmp)>pq{cmp};

	pq.push({robot_current_x*GRAPH_SIZE+robot_current_y,frame});

	while(!pq.empty()){
		auto &[point_hash,dict]=pq.top();
		pq.pop();

		
	}
}

int main() {
	// OPTIO;
	#ifdef DE_BUG
		// 用 cerr 输出调试信息
		OUTPUT = &std::cerr;
		// SEP = "\n";
		NEWLINE = true;
	#endif

	init();		// 初始化
	for (int i=1;i<=15000;i++) {
		get_input();	// 获取帧输入
		#ifdef DE_BUG
			debug(packet, robot, boat, money, frame, berth)
		#endif

		// 测试
		for(int i=0;i<10;i++)
			cout << "move " << i << " " << rand() % 4 << endl;
		cout << "OK" << endl;
	}

	return 0;
}

