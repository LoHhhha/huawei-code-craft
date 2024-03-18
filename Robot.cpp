#include "Param.hpp"
#include "Util.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"

void send_move(int id, int dir) { cout << "move " << id << " " << dir << endl; };
void send_get(int id) { cout << "get " << id << endl; };
void send_pull(int id) { cout << "pull " << id << endl; };

// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Robot& rb) {
	os << "机器人" << rb.id << ": 坐标(" << rb.x << ", " << rb.y << "), 正在拿的货物id: ";
	os << rb.packet_id << ", 运行状态: " << rb.status;
	os << ", 目标货物id: " << rb.target_packet_id;
	os << ", 目标泊位id: " << rb.target_berth_id;
	return os;
}
// ---------- end 重载输出流 ----------


// ---------- begin Robot方法实现 ----------

// 期望复杂度：1e6(4e4*1e2)
// 求解当前位置到每一点的最短可行路径 更新机器人shortest_dict/sleep 
// 描述的是绝对帧数
// 注意：update操作后，任一机器人对象的book操作都可能导致更新结果不正确
void Robot::update_dict() {
	for(int i=0;i<GRAPH_SIZE;i++) {
		for(int j=0;j<GRAPH_SIZE;j++) {
			this->shortest_dict[i][j] = INT_INF;
			this->sleep[i][j] = 1;	// 默认等待1秒 即将移动时间看作等待
		}
	}
	
	int robot_current_x = this->x, robot_current_y = this->y;
	this->shortest_dict[robot_current_x][robot_current_y] = frame;


	auto check_if_can_go = [&](int current_x, int current_y, int next_x, int next_y, int tframe) {
		// 判断是否碰墙/海/机器人
		if (graph[next_x][next_y]==-1 || graph[next_x][next_y]&ROBOT_BIT) {
			return false;
		}

		// 判断是否重叠
		auto next_it = book[next_x][next_y].find(tframe+1);
		if (next_it != book[next_x][next_y].end()) {
			return false;
		}

		// 判断是否对撞
		next_it = book[next_x][next_y].find(tframe);
		auto current_it = book[current_x][current_y].find(tframe+1);
		if (next_it!=book[next_x][next_y].end() && current_it!=book[current_x][current_y].end()) {
			return next_it->second != current_it->second;
		}

		return true;
	};

	// pii: {point_hash, tframe}
	auto cmp = [&](auto &p1, auto &p2) {
		return p1.second > p2.second;
	};
	priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(cmp)> pq{cmp};

	pq.push({robot_current_x*GRAPH_SIZE+robot_current_y, frame});

	while (!pq.empty()) {
		auto [point_hash, tframe] = pq.top();
		pq.pop();

		int point_x = point_hash/GRAPH_SIZE, point_y = point_hash%GRAPH_SIZE;

		if(tframe > this->shortest_dict[point_x][point_y]) {
			continue;
		}

		// 正常寻路
		for (auto &[dx, dy]:dir) {
			int next_x = point_x+dx, next_y = point_y+dy;
			if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {	// 越界
				continue;
			}

			if(tframe+1<this->shortest_dict[next_x][next_y] && check_if_can_go(point_x, point_y, next_x, next_y, tframe)) {
				this->shortest_dict[next_x][next_y] = tframe+1;
				pq.push({next_x*GRAPH_SIZE+next_y, tframe+1});
			}
		}

		// 等待后寻路
		// 等待的上界：book[point_x][point_y]第一个大于自己的帧数
		auto upper_it = book[point_x][point_y].upper_bound(tframe);
		int bound = INT_INF;
		if (upper_it != book[point_x][point_y].end()) {
			bound = upper_it->first;
		}
		for (auto &[dx,dy]:dir) {
			int next_x = point_x+dx, next_y = point_y+dy;
			if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {
				continue;
			}

			// 下一帧有人占用单元格，尝试等待
			auto tmp_it = book[next_x][next_y].find(tframe+1);
			int pos_next_frame = INT_INF;
			// 首先需要满足不越过边界，此操作最多校验1e2次
			while (tmp_it!=book[next_x][next_y].end() && tmp_it->first<bound) {
				if (check_if_can_go(point_x, point_y, next_x, next_y, tmp_it->first+1)) {
					pos_next_frame = tmp_it->first+1;
					break;
				}
			}
			if (pos_next_frame < this->shortest_dict[next_x][next_y]) {
				this->shortest_dict[next_x][next_y] = pos_next_frame;
				pq.push({next_x*GRAPH_SIZE+next_y, pos_next_frame});
				sleep[next_x][next_y] = pos_next_frame - tframe;
			}
		}
	}
}

// 期望复杂度：1
// 获取到某点的距离（需要的帧数），当距离为-1时不可达
// 注意：当前帧一定要调用过update_dict，否则结果不可信
int Robot::get_dict_to(int tx,int ty) {
	return this->shortest_dict[tx][ty]==INT_INF? -1 : this->shortest_dict[tx][ty];
}

// 期望复杂度：1e5(4e4*4)
// 设置机器人路径并订阅地图单元格
// 注意：此操作会调用get_dict_to，若不可达将返回false，使用时注意甄别
bool Robot::set_and_book_a_path_to(int tx, int ty) {
	int frame_arrive = get_dict_to(tx,ty);

	if (frame_arrive == -1) {
		fprintf(stderr, "#Warning: [%d]Robot::%d(%d,%d) fail to get path to (%d,%d) because unreachable.\n", frame, this->id, this->x, this->y, tx, ty);
		return false;
	}

	if(frame_arrive==frame){
		return true;
	}

	int current_x = tx, current_y = ty;
	while (current_x!=this->x || current_y!=this->y) {
		this->path.push({frame_arrive-1, current_x*GRAPH_SIZE+current_y});
		book[current_x][current_y][frame_arrive] = this->id;

		int isok = false;
		// 寻找上一个点
		for (auto &[dx,dy]:dir) {
			int pre_x = current_x+dx, pre_y = current_y+dy;
			if (pre_x>=GRAPH_SIZE || pre_y>=GRAPH_SIZE || pre_x<0 || pre_y<0) {	// 越界
				continue;
			}

			if (frame_arrive-this->sleep[current_x][current_y] == this->shortest_dict[pre_x][pre_y]) {
				current_x = pre_x, current_y = pre_y;
				frame_arrive = this->shortest_dict[pre_x][pre_y];
				isok = true;
				break;
			}
		}

		// 理论上不可能到达该处
		if (!isok) {
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) fail to set path to (%d,%d) because path cann`t get.\n",frame,this->id,this->x,this->y,tx,ty);

			// 清空book
			this->cancel_path_book();

			// 定义无目标
			this->target_berth_id=-1;
			
			return false;
		}
		
	}

	return true;
}

// 期望复杂度：1
// 指示机器人前往预定位置
// 注意：当没有路径时，返回false
bool Robot::go_to_next_point() {
	if (this->path.empty()) {
		fprintf(stderr,"#Warning: [%d]Robot::%d(%d,%d) do not have a target point.\n", frame, this->id, this->x, this->y);
		return false;
	}

	auto [frame_to_go, point_hash] = this->path.top();
	if (frame == frame_to_go) {
		int current_x = this->x, current_y = this->y;
		int next_x = point_hash/GRAPH_SIZE, next_y = point_hash%GRAPH_SIZE;
		bool isok = false;
		for (int i=0;i<4;i++) {
			auto &[dx, dy] = dir[i];
			if (current_x+dx==next_x && current_y+dy==next_y) {
				// printf(MOVE_OP, this->id, i);
				send_move(this->id, i);
				isok = true;
			}
		}
		if (!isok) {
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) fail to move.\n",frame,this->id,this->x,this->y);
		}
		this->path.pop();
	}
	else if(frame>frame_to_go){
		int current_x = this->x, current_y = this->y;
		int next_x = point_hash/GRAPH_SIZE, next_y = point_hash%GRAPH_SIZE;
		if(current_x!=next_x||current_y!=next_y){
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) path error because timeout.\n",frame,this->id,this->x,this->y);
			this->cancel_path_book();
		}
		this->path.pop();
	}

	return true;
}

// 撤销机器人已预订的路径
void Robot::cancel_path_book(){
	while (!this->path.empty()) {
		auto [rframe, point_hash] = this->path.top();
		this->path.pop();

		int p_x = point_hash/GRAPH_SIZE, p_y = point_hash%GRAPH_SIZE;
		book[p_x][p_y].erase(rframe+1);
	}
	this->target_berth_id=-1,this->target_packet_id=-1;
}

// 订阅一个取货物的事件
void Robot::book_get_packet_event(int arrive_frame){
	msg_handler.add_an_event(arrive_frame,this->id,MSG_ROBOT_NEED_GET);
}

// 订阅一个放货物的事件
void Robot::book_pull_packet_event(int arrive_frame){
	msg_handler.add_an_event(arrive_frame,this->id,MSG_ROBOT_NEED_PULL);
}

// 期望复杂度：1
// 指示机器人拿起货物，robot.target_packet_id=-1
// 注意：没有考虑是否有货物，一帧内切勿使用多次，到达目的地可以马上取
void Robot::get_packet(){
	// printf(GET_OP,this->id);
	send_get(this->id);
	this->target_packet_id=-1;
	take_packet(this->packet_id);
}

// 期望复杂度：1
// 指示机器人放下货物，robot.target_berth_id=-1
// 注意：没有考虑是否有货物，一帧内切勿使用多次，到达目的地可以马上取
void Robot::pull_packet(){
	// printf(PULL_OP,this->id);
	send_pull(this->id);
	berth[this->target_berth_id].current_wait_packet++;
	delete_packet(this->packet_id);
	this->target_berth_id=-1;
}

// 期望复杂度：1
// 如果机器人不处于恢复状态，该方法不生效
// 当机器人异常时通过此方法减少影响，默认cancel_path_book
// packet_id!=-1(是否拿着物品)	target_packet_id!=-1(是否有目标包裹)	target_berth_id!=-1(是否有目标泊位)
// 			0									0									0						什么都不做
// 			0									0									1						理论上不可能出现 target_berth_id=-1
// 			0									1									0						将货物预定放出
// 			0									1									1						理论上不可能出现 target_berth_id=-1, target_packet_id=-1
// 			1									0									0						什么都不做（重新规划路线在main/方法内）
// 			1									0									1						什么都不做（重新规划路线在main/方法内）
// 			1									1									0						理论上不可能出现 target_packet_id=-1
// 			1									1									1						理论上不可能出现 target_packet_id=-1
void Robot::recover(){
	if(this->status){
		fprintf(stderr,"#Warning: [%d]Robot::%d(%d,%d) do not need recover.\n", frame, this->id, this->x, this->y);
		return;
	}

	this->cancel_path_book();

	bool packet_id_exist=this->packet_id!=-1;
	bool target_packet_id_exist=this->target_packet_id!=-1;
	bool target_berth_id_exist=this->target_berth_id!=-1;
	int which=(packet_id_exist<<2)+(target_packet_id_exist<<1)+target_berth_id_exist;
	switch(which){
	case 0:
		break;
	case 1:
		this->target_berth_id=-1;
		break;
	case 2:
		packet_unbook(this->target_packet_id);
		this->target_packet_id=-1;
		break;
	case 3:
		this->target_berth_id=-1,this->target_packet_id=-1;
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		this->target_packet_id=-1;
		break;
	case 7:
		this->target_packet_id=-1;
		break;
	default:
		break;
	}
}

// 顶层函数(不必调用其余函数)
// 预期复杂度：1e6
// 取到货物时向最近出发
// 成功时返回true并设置robot.target_berth_id，否则false
// 注意：请在拿到货物后调用；最近的泊位根据的是机器人所在位置；调用后需要立即检查是否需要走
bool Robot::go_to_nearest_berth(){
	if(this->status==0||this->packet_id==-1||!this->path.empty()){
		return false;
	}

	this->update_dict();
	int nearest_berth=go_to_which_berth[this->x][this->y].first;

	// 不可能发生的情况
	if(nearest_berth==-1){
		fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) cann`t find a nearest berth.\n", frame, this->id, this->x, this->y);
		return false;
	}

	int point_hash=-1,nearest_dict=INT_INF;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			int current_dict=this->get_dict_to(i,j);
			if(current_dict!=-1&&current_dict<nearest_dict){
				nearest_dict=current_dict;
				point_hash=i*GRAPH_SIZE+j;
			}
		}
	}
	// 暂时不可达
	if(point_hash==-1){
		return false;
	}

	int target_point_x=point_hash/GRAPH_SIZE,target_point_y=point_hash%GRAPH_SIZE;
	this->set_and_book_a_path_to(target_point_x,target_point_y);
	this->target_berth_id=nearest_berth;
	this->book_pull_packet_event(this->shortest_dict[target_point_x][target_point_y]);
    return true;
}

// 顶层函数(不必调用其余函数)
// 预期复杂度：1e6
// 寻找最好的包裹
// 更换目标时返回true并设置robot.target_packet_id，否则false
// 具体算法：寻找value/dict最大的包裹
// 注意：本函数可多次调用【只要未拿到包裹可随时更新最优】
bool Robot::find_a_best_packet(){
	if(this->status==0||this->packet_id!=-1){
		return false;
	}

	this->update_dict();

	int best_packet_hash=-1;
	double best_rate=-1;
	for(int i=0;i<GRAPH_SIZE;i++){
		for(int j=0;j<GRAPH_SIZE;j++){
			int packet_hash=i*GRAPH_SIZE+j;
			// 抵达货物时不会超时
			if(graph[i][j]&PACKET_BIT&&packet[hash2packet[packet_hash]].timeout>this->shortest_dict[i][j]){
				double current_rate=(packet[hash2packet[packet_hash]].value)/(this->shortest_dict[i][j]-frame+go_to_which_berth[i][j].second);
				if(current_rate==best_rate){
					best_rate=current_rate;
					best_packet_hash=packet_hash;
				}
			}
		}
	}

	// 当更好的不是当前算到最好的则修改
	int new_target_packet_id=hash2packet[best_packet_hash];
	if(this->target_packet_id!=new_target_packet_id){
		int best_packet_x=best_packet_hash/GRAPH_SIZE,best_packet_y=best_packet_hash%GRAPH_SIZE;
		this->cancel_path_book();
		this->set_and_book_a_path_to(best_packet_x,best_packet_y);
		this->target_packet_id=new_target_packet_id;
		this->book_get_packet_event(this->shortest_dict[best_packet_x][best_packet_y]);
		return true;
	}
	return false;
}
// ---------- end Robot方法实现 ----------