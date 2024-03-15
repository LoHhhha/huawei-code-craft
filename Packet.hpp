#pragma once

#include "util.h"
#include "Robot.hpp"

struct Packet {
	int id;			// 货物id
	int x, y;		// 货物位置
	int value;		// 货物价值
	int timeout;	// 过期时间：帧

	Packet() = default;
	Packet(int _id, int _x, int _y, int _value, int _timeout) : id(_id), x(_x), y(_y), value(_value), timeout(_timeout) {}
	
	void broadcast();

	friend ostream& operator<<(ostream& os, const Packet& packet);
};
int packet_id = -1;			// 当前最后一个货物id
map<int, Packet> packet;	// 货物

// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Packet& packet) {
	os << "货物: id: " << packet.id << ", 坐标(" << packet.x << ", " << packet.y << "), ";
	os << "价值: " << packet.value << ", 过期帧: " << packet.timeout;
	return os;
}
// ---------- end 重载输出流 ----------


// ---------- begin Packet方法实现 ----------

// 每当有货物生成时调用该方法，向附近最合适的机器人广播货物信息
void Packet::broadcast() {
	queue<int> qu;	// point_hash
	qu.push(this->x*GRAPH_SIZE+this->y);

	int step = 0;
	bool isok = false;
	while (!qu.empty()) {
		int qn = qu.size();
		while (qn--) {
			auto point_hash = qu.front(); qu.pop();
			
			int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
			for (auto &[tx, ty]:dir) {
				int next_x = current_x+tx, next_y = current_y+ty;
				if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {	// 越界
					continue;
				}
				if (graph[next_x][next_y] == -1) {	// 障碍
					continue;
				}
				if (step+1 >= 1000) {	// 超时
					isok = true;
					break;
				}
				if (graph[next_x][next_y] & ROBOT_BIT) {	// 机器人
					auto &rb = robot[0];
					for (auto &it:robot) {	// 找到对应机器人
						if (it.x == next_x && it.y == next_y) {
							rb = it;
						}
					}
					if (rb.status == 0) {	// 机器人处于恢复状态
						continue;
					}
					if (rb.packet_id == -1 && rb.path.empty()) {	// 没有要取的物品，直接把货物分配给该机器人
						rb.packet_id = this->id;
						rb.target_berth_id = -1;
						rb.update_dict();	// 更新最短路径
						bool can_arrive = rb.set_and_book_a_path_to(this->x, this->y);	// 设置路径
						// !正常不会出现
						if (!can_arrive) {
							fprintf(stderr,"#Error: [%d]Packet::%d(%d,%d) fail to set path to (%d,%d).\n",frame,this->id,this->x,this->y,rb.x,rb.y);
						}
						isok = true;
						break;
					} else if (rb.packet_id == -1) {	// 有将要取的物品，已经规划好了路径，判断是否将货物重新分配给他
						int val1 = this->value;	// 当前货物价值
						int val2 = packet[rb.packet_id].value;	// 将要取的货物价值
						int t1 = rb.get_dict_to(this->x, this->y) - frame;	// 机器人到达当前货物所需时间
						int t2 = rb.get_dict_to(packet[rb.packet_id].x, packet[rb.packet_id].y) - frame;	// 机器人到原货物所需时间）
						t1 += go_to_which_berth[this->x][this->y].second;	// 当前货物到达泊位所需时间
						t2 += go_to_which_berth[packet[rb.packet_id].x][packet[rb.packet_id].y].second;	// 原货物到达泊位所需时间

						// todo: 待判断是否选取当前货物
					}
				}
				qu.push(next_x*GRAPH_SIZE+next_y);
			}
			if (isok) {
				break;
			}
		}
		if (isok) {
			break;
		}
		step++;
	}
}


// ---------- end Packet方法实现 ----------