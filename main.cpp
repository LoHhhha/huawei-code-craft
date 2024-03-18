#include "Param.hpp"
#include "Util.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"


// #define DE_BUG
#ifdef DE_BUG
    #include "DEBUG.h"
    using namespace DEBUG_;
	#define FRAME_TO_RUN 1
#else
	#define FRAME_TO_RUN FRAME_COUNT
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
	get_robot_can_go();
	choose_best_berth(min(BOAT_NUM,BERTH_NUM));

	cout << "OK" << endl;
}

// 获取帧输入，返回当前帧新生成多少个货物
int get_input() {
	cin >> frame >> money;	// 获取帧数和金钱数

	// 货物
	int goods_num;
	int real_goods_num = 0;
	cin >> goods_num;
	for (int i=0;i<goods_num;i++) {
		int x, y, packet_money;
		cin >> x >> y >> packet_money;
		// Packet p(++packet_id, x, y, packet_money, frame + PACKET_TIME_OUT);	// 在 1000 帧后过期
		// packet[packet_id] = p;
		real_goods_num += generate_packet(x, y, packet_money);	// 生成货物
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

	return real_goods_num;
}

// 检查是否存在异常状态（机器人是否处于恢复状态）并处理、船是否已满、新货物广播
void checker(){
	// 机器人处理
	// 不太可能出现
	for(int i=0;i<ROBOT_NUM;i++){
		if(!robot[i].status){
			fprintf(stderr, "#Error: [%d]Checker:: Robot::%d detect a crush.\n", frame, i);
			robot[i].recover();
		}
	}


	// 船检查是否已满
	for(int i=0;i<BOAT_NUM;i++){
		if(boat[i].load>=boat[i].capacity){
			boat[i].load=0;
			boat[i].deliver();
			msg_handler.add_an_event(frame+berth[boat[i].berth_id].transport_time,i,MSG_BOAT_NEED_BACK);
			if(boat[i].berth_id==-1){
				fprintf(stderr, "#Warning: [%d]Checker:: Boat::%d full but not in any berth.\n", frame, i);
			}
		}
	}

}

// 处理每一帧
void solve(){
	// 1. get_input 更新参数、新货物广播
	// 2. checker 检查是否存在异常状态（机器人是否处于恢复状态）并处理、船是否已满
	// 3. 空闲机器人重新规划路线\前往待机点\待机
	// 4. 机器人前进检查
	// 5. 消息处理
	// 6. 更新船只装载

	// step 1
	int goods_num = get_input();	// 获取帧输入
	for (int i=packet_id-goods_num+1;i<=packet_id;i++) {	// 广播新生成的货物（在结束帧输入后进行）
		broadcast_packet(i);
	}

	// step 2
	checker();

	// step 3
	for(int i=0;i<ROBOT_NUM;i++){
		if(robot[i].packet_id==-1&&robot[i].target_packet_id==-1){
			robot[i].find_a_best_packet();
		}
	}
	for(int i=0;i<ROBOT_NUM;i++){
		robot[i].go_to_nearest_berth();
	}
	// todo 空闲机器人的处理
	// 机器人均匀到达地图的点

	// step 4
	for(int i=0;i<ROBOT_NUM;i++){
		robot[i].go_to_next_point();
	}

	// step 5
	msg_handler.check_and_do();

	// step 6
	for(int i=0;i<BOAT_NUM;i++){
		if(boat[i].status==1&&boat[i].berth_id!=-1){
			auto &tberth=berth[boat[i].berth_id];
			int change_size=min(tberth.current_wait_packet,tberth.transport_time);
			boat[i].load+=change_size;
			tberth.current_wait_packet-=change_size;
		}
	}

	cout<<"OK"<<endl;
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
void solve_test_Packet_broadcast() {
	int goods_num = get_input();	// 获取帧输入
	// auto pk = packet.begin()->second;
	// bool flag = pk.broadcast();
	for (auto &[id, pk]:packet) {
		pk.broadcast(); 
	}
	#ifdef DE_BUG
		debug(packet, robot)
	#endif
}

int main() {
	#ifdef DE_BUG
		// SEP = "\n"	// 输出分隔符，默认为"  "，不会作用于容器内的对象
		NEWLINE = true;	// 是否换行，会作用于容器内的对象，覆盖SEP
	#endif
	#if (DEBUG_STATE == 1)
		freopen("judge_output.txt", "r", stdin);
		// freopen("debug/user_output.txt", "w", stdout);
	#endif
	freopen("debug/debug_output.txt", "w", stderr);

	init();		// 初始化
	for (int i=1;i<=FRAME_TO_RUN;i++) {
		solve_test_Packet_broadcast();
	}

	return 0;
}

