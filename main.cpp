#include "Param.hpp"
#include "Util.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"


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

	// #ifdef DE_BUG
	// 	debug(graph, robot_can_go, go_to_which_berth)
	// #endif

	cout << "OK" << endl << flush;
	
	// 安排船驻扎
	// 注册最后离港时间
	for(int i=0,boat_idx=0;i<BERTH_NUM;i++){
		if(use_berth[i]){
			boat[boat_idx].bind_berth_id=i;
			boat[boat_idx].go_to_berth(i);
			msg_handler.add_an_event(FRAME_COUNT-berth[i].transport_time,boat_idx,MSG_BOAT_NEED_GO);
			boat_idx++;
		}
	}
}

// 获取帧输入，返回当前帧新生成多少个货物
int get_input() {
	int tmp_frame;
	cin >> tmp_frame >> money;	// 获取帧数和金钱数

	if(tmp_frame!=frame+1){
		fprintf(stderr, "#Error(main::get_input): [%d]TimeOut, %d.\n", frame, tmp_frame);
		exit(-1);
	}

	frame=tmp_frame;

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
		else{
			if(robot[i].target_packet_id==-1){
				fprintf(stderr, "#Warning(main::get_input): [%d]Robot::%d packet information disappear.\n", frame, i);
				robot[i].packet_id = 0;				// 补全
			}
			else{
				robot[i].packet_id = robot[i].target_packet_id;
				robot[i].target_packet_id=-1;
			}
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
	// vector<Packet*> new_packet;
	// for (int i=packet_id-goods_num+1;i<=packet_id;i++) {	// 广播新生成的货物（在结束帧输入后进行）
	// 	new_packet.push_back(&packet[i]);
	// }
	// sort(new_packet.begin(), new_packet.end(), [&](Packet *a, Packet *b) {	// 按照货物价值排序，减少换货物导致多次update的情况
	// 	return a->value > b->value;
	// });
	// for (auto &pk:new_packet) {
	// 	pk->broadcast();
	// }

	// #ifdef DE_BUG
	// 	debug(packet)
	// #endif

	// step 2
	checker();

	// step 3
	for(int i=0;i<ROBOT_NUM;i++){
		robot[i].find_a_best_packet();
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

	cout << "OK" << endl << flush;
}


int main() {
	// ios::sync_with_stdio(0), cin.tie(0), cout.tie(0);
	#ifdef DE_BUG
		// SEP = "\n";	// 输出分隔符，默认为"  "，不会作用于容器内的对象
		// NEWLINE = true;	// 是否换行，会作用于容器内的对象，覆盖SEP
	#endif
	#if (THIS_DEBUG_STATE == 1)	// 调试模式： 0：关闭，1：终端，2：cph
		freopen("z_judge_output1.txt", "r", stdin);
		freopen("user_output.txt", "w", stdout);
	#endif
	freopen("debug_output.txt", "w", stderr);	// 重定向错误流
	#ifndef ENABLE_STDERR
		fprintf(stderr,"ENABLE_STDERR=false.\n");
		freopen("/dev/null", "w", stderr);	// 重定向错误流
	#endif


	clock_t start,end;
	start=clock();
	init();		// 初始化
	end=clock();
	fprintf(stderr,"#Note(main::init): Init Using %fms.\n\n",double(end-start));

	for (int i=1;i<=FRAME_TO_RUN;i++) {
		start=clock();
		solve();
		end=clock();
		fprintf(stderr,"#Note(main::solve): [%d]Using %fms.\n\n",frame,double(end-start));
		cout<<double(end-start)<<endl;
	}

	return 0;
}

// ----------------- test -----------------

// // 处理每一帧
// void solve_test() {
// 	get_input();	// 获取帧输入
// 	robot[0].update_dict();
// 	for (auto [id, pk]:packet) {
// 		printf("货物id: %d, (%d, %d), %d\n", id, pk.x, pk.y, robot[0].get_dict_to(pk.x, pk.y));
// 	}
// 	auto pk = packet.begin()->second;
// 	robot[0].set_and_book_a_path_to(pk.x, pk.y);
// 	#ifdef DE_BUG
// 		OUTPUT = &std::cout;
// 		debug(robot[0].path)
// 	#endif
// 	robot[0].go_to_next_point();


// 	// #ifdef DE_BUG
// 	// 	debug(robot[0], robot[0].path, robot[0].shortest_dict, robot[0].sleep)
// 	// #endif
// }

// // 处理每一帧
// void solve_test_Packet_broadcast() {
// 	int goods_num = get_input();	// 获取帧输入
// 	// auto pk = packet.begin()->second;
// 	// bool flag = pk.broadcast();
// 	for (auto &[id, pk]:packet) {
// 		pk.broadcast(); 
// 	}
// 	#ifdef DE_BUG
// 		debug(packet, robot)
// 	#endif
// }

// // ----------------- test -----------------