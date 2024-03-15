#include "util_function.h"

#define DE_BUG
#ifdef DE_BUG
    #include "DEBUG.h"
    using namespace DEBUG_;
	#define FRAME_CNT 1
#else
	#define FRAME_CNT 15000
#endif

#define DEBUG_STATE 1	// 0：关闭，1：终端，2：cph


// 初始化
void init() {
	// 地图初始化
	auto f = [](char c) {
		if (c == '#' || c == '*') return -1;	// 障碍或海洋
		else if (c == '.') return 0;			// 空地
		else if (c == 'B') return BERTH_BIT;	// 泊位
		else if (c == 'A') return ROBOT_BIT;	// 机器人
		else return -100;						// 输入有误
	};
	int robot_id = 0;	// 机器人编号
	for (int i=0;i<GRAPH_SIZE;i++) {
		for (int j=0;j<GRAPH_SIZE;j++) {
			int cell = f(cin.get());
			if (cell == ROBOT_BIT) {	// 初始化机器人
				robot[robot_id] = Robot(robot_id, i, j, 1);
				robot_id++;
			}
			graph[i][j] = cell;
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
	for(int boat_id=0;boat_id<BOAT_NUM;boat_id++) {
		boat[boat_id] = Boat(boat_id, 1, -1, 0, boat_capacity);
	}
	string okk;
	cin >> okk;

	// 在这里写初始化代码
	choose_best_berth(5);

	cout << "OK" << endl;
}

// 获取帧输入，返回当前帧新生成多少个货物
int get_input() {
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
		if (!have_packet) {
			robot[i].packet_id = -1;
		}

		graph[robot[i].x][robot[i].y] ^= ROBOT_BIT;	// 机器人上一帧位置清空
		graph[x][y] ^= ROBOT_BIT;					// 机器人当前帧位置标记

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

	return goods_num;
}

// 处理每一帧
void solve_test() {
	get_input();	// 获取帧输入
	robot[0].update_dict();
	for (auto [id, pk]:packet) {
		printf("货物id: %d, (%d, %d), %d\n", id, pk.x, pk.y, robot[0].get_dict_to(pk.x, pk.y));
	}
	auto pk = packet.begin()->second;
	robot[0].set_and_book_a_path_to(pk.x, pk.y);
	#ifdef DE_BUG
		OUTPUT = &std::cout;
		debug(robot[0].path)
	#endif
	robot[0].go_to_next_point();


	// #ifdef DE_BUG
	// 	debug(robot[0], robot[0].path, robot[0].shortest_dict, robot[0].sleep)
	// #endif
}

// 处理每一帧
void solve() {
	int goods_num = get_input();	// 获取帧输入
}

int main() {
	#ifdef DE_BUG
		// NEWLINE = true;
	#endif
	#if (DEBUG_STATE == 1)
		freopen("judge_output.txt", "r", stdin);
		// freopen("debug/user_output.txt", "w", stdout);
	#endif

	init();		// 初始化
	for (int i=1;i<=FRAME_CNT;i++) {
		solve();
	}

	return 0;
}

