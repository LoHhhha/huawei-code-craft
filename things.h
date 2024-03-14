#include<bits/stdc++.h>
using namespace std;

using ll = long long;
using pii = pair<int, int>;

// #define int ll
// #define endl "\n"	// 是否保留待定
#define OPTIO std::ios::sync_with_stdio(0), std::cin.tie(0), std::cout.tie(0)
#define LLONG_INF 0x3f3f3f3f3f3f3f3f
#define INT_INF 0x3f3f3f3f
#define GRAPH_SIZE 200	// 地图大小
#define ROBOT_NUM 	10	// 机器人数量
#define BERTH_NUM 	10	// 泊位数量
#define BOAT_NUM 	5	// 船数量
#define RIGHT 		0	// 机器人方向：右
#define LEFT 		1	// 机器人方向：左
#define UP 			2	// 机器人方向：上
#define DOWN 		3	// 机器人方向：下
#define BERTH_BIT 	1	// 泊位标记
#define ROBOT_BIT 	2	// 机器人标记
#define PACKET_BIT 	4	// 货物标记

#define MOVE_OP "move %d %d"		// 机器人移动指令 move id[0-9] dir[0-3]
#define GET_OP "get %d"				// 机器人取货指令 get id[0-9]
#define PULL_OP "pull %d"			// 机器人放货指令 pull id[0-9]

int dir[4][2] = {{0, 1},{0, -1},{-1, 0},{1, 0}};		// 移动方向，下标与机器人指令方向一致


struct Robot {
	int id;
	int x, y;										// 机器人当前坐标
	int status;										// 0: 恢复状态 1: 正常运行状态
	int packet_id;									// -1: 无货物 其他整数: 货物编号
	int target_berth_id;							// -1：无目的泊位 其他整数: 泊位编号
	int shortest_dict[GRAPH_SIZE][GRAPH_SIZE]{0};	// 最短路记录矩阵，与update_dict / get_dict_to / get_and_book_a_path_to一并使用
	int sleep[GRAPH_SIZE][GRAPH_SIZE]{0};			// 最短路等待记录矩阵，描述的是【去】该格等待的时间，与update_dict / get_dict_to / get_and_book_a_path_to一并使用
	stack<pii> path;								// 维护机器人路径{frame_to_go（出发时间）, point_hash}，go_to_next_point

	void update_dict();
	int get_dict_to(int tx,int ty);
	bool set_and_book_a_path_to(int tx,int ty);
	bool go_to_next_point();

	void get_packet();
	void pull_packet();

	friend ostream& operator<<(ostream& os, const Robot& robot);
};
vector<Robot> robot(ROBOT_NUM);	// 机器人 vector


struct Berth {
	int x, y;			// 码头左上角坐标
	int transport_time;	// 运输到虚拟点的时间
	int loading_speed;	// 装载速度: 个/帧

	friend ostream& operator<<(ostream& os, const Berth& berth);
};
vector<Berth> berth(BERTH_NUM);	// 码头 vector


struct Packet {
	int id;			// 货物id
	int x, y;		// 货物位置
	int value;		// 货物价值
	int timeout;	// 过期时间：帧

	Packet() = default;
	Packet(int _id, int _x, int _y, int _value, int _timeout) : id(_id), x(_x), y(_y), value(_value), timeout(_timeout) {}
	friend ostream& operator<<(ostream& os, const Packet& packet);
};
int packet_id = -1;			// 当前最后一个货物id
map<int, Packet> packet;	// 货物


struct Boat {
    int status;         // 0：移动中，1：正常运行状态(装货/运输完成状态)，2：泊位外等待状态
    int berth_id;	    // 目标泊位id，值为-1时表示目标泊位为虚拟点
    int load;           // 目前装载数
    int capacity;  		// 船的容量 *初赛固定

	friend ostream& operator<<(ostream& os, const Boat& boat);
};
vector<Boat> boat(BOAT_NUM);	// 船 vector


vector<vector<int>> graph(GRAPH_SIZE, vector<int>(GRAPH_SIZE));	// 地图 vector 障碍:-1 空地:0 停泊点:1 机器人:2 货物:4 （二进制）

vector<vector<map<int,int>>> book(GRAPH_SIZE, vector<map<int,int>>(GRAPH_SIZE));	// pii:{book_frame,id} 点被预定的情况

int frame;	// 当前帧数

int money;	// 当前金钱数

// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Robot& rb) {
	os << "机器人" << rb.id << ": 坐标(" << rb.x << ", " << rb.y << "), 货物id: ";
	os << rb.packet_id << ", 运行状态: " << rb.status;
	os << ", 目标货物id: " << rb.target_berth_id;
	os << ", 目标泊位id: " << rb.target_berth_id;
	return os;
}

ostream& operator<<(ostream& os, const Berth& berth) {
	os << "泊位: 坐标(" << berth.x << ", " << berth.y << "), 运输时间: ";
	os << berth.transport_time << ", 装载速度: " << berth.loading_speed;
	return os;
}

ostream& operator<<(ostream& os, const Packet& packet) {
	os << "货物: id: " << packet.id << ", 坐标(" << packet.x << ", " << packet.y << "), ";
	os << "价值: " << packet.value << ", 过期帧: " << packet.timeout;
	return os;
}

ostream& operator<<(ostream& os, const Boat& boat) {
	os << "船: 状态: " << boat.status << ", 目标泊位id: " << boat.berth_id << ", ";
	os << "装载数: " << boat.load << ", 容量: " << boat.capacity;
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
		auto &[point_hash, tframe] = pq.top();
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
				isok = true;
				break;
			}
		}

		// 理论上不可能到达该处
		if (!isok) {
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) fail to set path to (%d,%d).\n",frame,this->id,this->x,this->y,tx,ty);
			// 清空book
			while (!this->path.empty()) {
				auto [rframe, point_hash] = this->path.top();
				this->path.pop();

				int p_x = point_hash/GRAPH_SIZE, p_y = point_hash%GRAPH_SIZE;
				book[p_x][p_y].erase(rframe+1);
			}
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
		fprintf(stderr,"#Warning: [%d]Robot::%d(%d,%d) do not have a target point.\n",frame,this->id,this->x,this->y);
		return false;
	}

	auto [frame_to_go, point_hash] = this->path.top();
	if (frame == frame_to_go) {
		int current_x = this->x, current_y = this->y;
		int next_x = point_hash/GRAPH_SIZE, next_y = point_hash%GRAPH_SIZE;
		bool isok = false;
		for(int i=0;i<4;i++) {
			auto &[dx,dy] = dir[i];
			if (current_x+dx==next_x && current_y+dy==next_y) {
				printf(MOVE_OP, this->id, i);
				isok=true;
			}
		}
		if(!isok){
			fprintf(stderr,"#Error: [%d]Robot::%d(%d,%d) fail to move.\n",frame,this->id,this->x,this->y);
		}
		this->path.pop();
	}

	return true;
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