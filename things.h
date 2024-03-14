#include<bits/stdc++.h>
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
#define RIGHT 	0
#define LEFT 	1
#define UP 		2
#define DOWN 	3

int dir[4][2]={{0,1},{1,0},{-1,0},{0,-1}};


struct Robot {
	int x, y;	// 机器人当前坐标
	int packet_id;	// 0: 无货物 其他整数: 货物编号
	int status;	// 0: 恢复状态 1: 正常运行状态
	int shortest_dict[GRAPH_SIZE][GRAPH_SIZE];
	friend ostream& operator<<(ostream& os, const Robot& robot);
	void update_dict();
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
int packet_id = 0;			// 当前最后一个货物id
map<int, Packet> packet;	// 货物


struct Boat {
    int status;         // 0：移动中，1：正常运行状态(装货/运输完成状态)，2：泊位外等待状态
    int berth_id;	    // 目标泊位id，值为-1时表示目标泊位为虚拟点
    int load;           // 目前装载数
    int capacity;  // 船的容量 *初赛固定

	friend ostream& operator<<(ostream& os, const Boat& boat);
};
vector<Boat> boat(BOAT_NUM);	// 船 vector


vector<vector<int>> graph(GRAPH_SIZE, vector<int>(GRAPH_SIZE));	// 地图 vector 障碍:-1 空地:0 停泊点:1 货物:2 机器人:3

vector<vector<map<int,int>>> book(GRAPH_SIZE, vector<map<int,int>>(GRAPH_SIZE));	// pii:{book_frame,id} 点被预定的情况

int frame;	// 当前帧数

int money;	// 当前金钱数


ostream& operator<<(ostream& os, const Robot& robot) {
	os << "机器人: 坐标(" << robot.x << ", " << robot.y << "), 货物id: ";
	os << robot.packet_id << ", 运行状态: " << robot.status;
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


// 期望复杂度：1e6(4e4*1e2)
// 更新机器人shortest_dict
void Robot::update_dict(){
	for(int i=0;i<GRAPH_SIZE;i++){
		for(int j=0;j<GRAPH_SIZE;j++){
			this->shortest_dict[i][j] = INT_INF;
		}
	}
	
	int robot_current_x = this->x, robot_current_y = this->y;
	this->shortest_dict[robot_current_x][robot_current_y] = frame;

	auto check_if_can_go=[&](int current_x,int current_y,int next_x,int next_y,int tframe){
		// 判断是否碰墙/机器人/海
		if(graph[next_x][next_y]!=0&&graph[next_x][next_y]!=1){
			return false;
		}

		// 判断是否重叠
		auto next_it=book[next_x][next_y].find(tframe+1);
		if(next_it!=book[next_x][next_y].end()){
			return false;
		}

		// 判断是否对撞
		next_it=book[next_x][next_y].find(tframe);
		auto current_it=book[next_x][next_y].find(tframe+1);
		if(next_it!=book[next_x][next_y].end()&&current_it!=book[next_x][next_y].end()){
			return next_it->second!=current_it->second;
		}

		return true;
	};

	// pii: {point_hash,tframe}
	auto cmp = [&](auto &p1, auto &p2){
		return p1.second>p2.second;
	};
	priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(cmp)> pq{cmp};

	pq.push({robot_current_x*GRAPH_SIZE+robot_current_y,frame});

	while(!pq.empty()){
		auto &[point_hash,tframe] = pq.top();
		pq.pop();

		int point_x=point_hash/GRAPH_SIZE,point_y=point_hash%GRAPH_SIZE;

		if(tframe>this->shortest_dict[point_x][point_y]){
			continue;
		}

		// 正常寻路
		for(auto &[dx,dy]:dir){
			int next_x=point_x+dx,next_y=point_y+dy;
			if(next_x>=GRAPH_SIZE||next_y>=GRAPH_SIZE||next_x<0||next_y<0){
				continue;
			}

			if(tframe+1<this->shortest_dict[next_x][next_y]&&check_if_can_go(point_x,point_y,next_x,next_y,tframe)){
				pq.push({next_x*GRAPH_SIZE+next_y,tframe+1});
			}
		}

		// 等待后寻路
		// 等待的上界：book[point_x][point_y]第一个大于自己的帧数
		auto upper_it=book[point_x][point_y].upper_bound(tframe);
		int bound=INT_INF;
		if(upper_it!=book[point_x][point_y].end()){
			bound=upper_it->first;
		}
		for(auto &[dx,dy]:dir){
			int next_x=point_x+dx,next_y=point_y+dy;
			if(next_x>=GRAPH_SIZE||next_y>=GRAPH_SIZE||next_x<0||next_y<0){
				continue;
			}

			// 下一帧有人占用单元格，尝试等待
			auto tmp_it=book[next_x][next_y].find(tframe+1);
			int pos_next_frame=INT_INF;
			// 首先需要满足不越过边界，此操作最多校验1e2次
			while(tmp_it!=book[next_x][next_y].end()&&tmp_it->first<bound){
				if(check_if_can_go(point_x,point_y,next_x,next_y,tmp_it->first+1)){
					pos_next_frame=tmp_it->first+1;
					break;
				}
			}
			if(pos_next_frame<this->shortest_dict[next_x][next_y]){
				pq.push({next_x*GRAPH_SIZE+next_y,pos_next_frame});
			}
		}
	}
}