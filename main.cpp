#include<bits/stdc++.h>
#include"things.h"
using namespace std;

using ll = long long;
using pii = pair<int, int>;

// #define int ll
#define endl "\n"	// 是否保留待定
#define OPTIO std::ios::sync_with_stdio(0), std::cin.tie(0), std::cout.tie(0)
#define INF 0x3f3f3f3f3f3f3f3f
#define inf 0x3f3f3f3f

#define DE_BUG
#ifdef DE_BUG
    #include "DEBUG.h"
    using namespace DEBUG_;
#endif

#define RIGHT 	0
#define LEFT 	1
#define UP 		2
#define DOWN 	3


vector<Robot> robot(10);	// 机器人 vector

vector<Berth> berth(10);	// 码头 vector

int packet_id = 0;			// 当前最后一个货物id
map<int, Packet> packet;	// 货物

vector<vector<int>> graph(200, vector<int>(200));	// 地图 vector

vector<Boat> boat(5);	// 船 vector

vector<vector<set<int>>> book(200, vector<set<int>>(200));	// 点被预定的情况

int frame;	// 帧数

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
	for (int i=0;i<200;i++) {
		for (int j=0;j<200;j++) {
			graph[i][j] = f(cin.get());
		}
		cin.get();
	}

	// 泊位初始化
	for (int i=0;i<10;i++) {
		cin >> berth[i].x >> berth[i].y >> berth[i].transport_time >> berth[i].loading_speed;
	}

	int boat_capacity;	// 船的容量
	cin >> boat_capacity;	
	for(int i=0;i<5;i++) {
		boat[i].load = 0;
		boat[i].status = 2;
		boat[i].berth_id = -1;
		boat[i].capacity = boat_capacity;
	}
	string okk;
	cin >> okk;

	// 在这里写初始化代码

	cout << "OK" << endl << flush;
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
	for (int i=0;i<10;i++) {
		int have_packet, x, y, robot_status;	// have_packet表示机器人是否有货物，Robot内packet_id表示货物id
		cin >> have_packet >> x >> y >> robot_status;
		// todo：packet_id待处理
		robot[i].x = x;
		robot[i].y = y;
		robot[i].status = robot_status;
	}

	// 船
	for (int i=0;i<5;i++) {
		int boat_status, boat_target;
		cin >> boat_status >> boat_target;
		boat[i].status = boat_status;
		boat[i].berth_id = boat_target;
	}

	string okk;
	cin >> okk;
}


int get_and_set_a_path(int robot_id,int x,int y){
	// point_hash, dict
	queue<pair<int,int>>qu;
	

	return 0;
}

int main() {
	OPTIO;
	#ifdef DE_BUG
		// 用 cerr 输出调试信息
		OUTPUT = &std::cerr;
		SEP = "\n";
		NEWLINE = true;
	#endif

	init();		// 初始化
	for (int i=0;i<1;i++) {
		get_input();	// 获取帧输入
		#ifdef DE_BUG
			debug(packet, robot, boat, money, frame, berth)
		#endif
	}

	return 0;
}

