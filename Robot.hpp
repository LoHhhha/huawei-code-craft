#pragma once

#include "util.h"

#define MOVE_OP "move %d %d"		// 机器人移动指令 move id[0-9] dir[0-3]
#define GET_OP "get %d"				// 机器人取货指令 get id[0-9]
#define PULL_OP "pull %d"			// 机器人放货指令 pull id[0-9]


struct Robot {
	int id;											// 机器人编号
	int x, y;										// 机器人当前坐标
	int status;										// 0: 恢复状态 1: 正常运行状态
	int packet_id;									// -1: 无货物 其他整数: 货物编号
	int target_berth_id;							// -1：无目的泊位 其他整数: 泊位编号
	int target_packet_id;							// -1：无目的货物 其他整数: 货物编号
	int shortest_dict[GRAPH_SIZE][GRAPH_SIZE]{0};	// 最短路记录矩阵，与update_dict / get_dict_to / get_and_book_a_path_to一并使用
	int sleep[GRAPH_SIZE][GRAPH_SIZE]{0};			// 最短路等待记录矩阵，描述的是【去】该格等待的时间，与update_dict / get_dict_to / get_and_book_a_path_to一并使用
	stack<pii> path;								// 维护机器人路径{frame_to_go（出发时间）, point_hash}，go_to_next_point

	Robot() = default;
	Robot(int id, int x, int y, int status): id(id), x(x), y(y), status(status), target_berth_id(-1), target_packet_id(-1) {}

	void update_dict();
	int get_dict_to(int tx,int ty);
	bool set_and_book_a_path_to(int tx,int ty);
	void cancel_path_book();
	bool go_to_next_point();

	void get_packet();
	void pull_packet();

	friend ostream& operator<<(ostream& os, const Robot& robot);
};
vector<Robot> robot(ROBOT_NUM);	// 机器人 vector

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
		fprintf(stderr, "#Warning: [%d]Robot::%d(%d,%d) fail to get path to (%d,%d).\n", frame, this->id, this->x, this->y, x, y);
		return false;
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
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) fail to set path to (%d,%d).\n",frame,this->id,this->x,this->y,tx,ty);

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
				printf(MOVE_OP, this->id, i);
				isok = true;
			}
		}
		if (!isok) {
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) fail to move.\n",frame,this->id,this->x,this->y);
		}
		this->path.pop();
	}

	return true;
}

void Robot::cancel_path_book(){
	while (!this->path.empty()) {
		auto [rframe, point_hash] = this->path.top();
		this->path.pop();

		int p_x = point_hash/GRAPH_SIZE, p_y = point_hash%GRAPH_SIZE;
		book[p_x][p_y].erase(rframe+1);
	}
}

// 期望复杂度：1
// 指示机器人拿起货物
// 注意：没有考虑是否有货物，一帧内切勿使用多次，到达目的地可以马上取
void Robot::get_packet(){
	printf(GET_OP,this->id);
}

// 期望复杂度：1
// 指示机器人放下货物
// 注意：没有考虑是否有货物，一帧内切勿使用多次，到达目的地可以马上取
void Robot::pull_packet(){
	printf(PULL_OP,this->id);
}

// ---------- end Robot方法实现 ----------