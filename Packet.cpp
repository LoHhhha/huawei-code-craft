#include "Param.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"


// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Packet& packet) {
	os << "货物: id: " << packet.id << ", 坐标(" << packet.x << ", " << packet.y << "), ";
	os << "价值: " << packet.value << ", 过期帧: " << packet.timeout;
	os << ", 状态: " << packet.status;
	return os;
}
// ---------- end 重载输出流 ----------


// ---------- begin Packet方法实现 ----------

// 每当有货物生成时调用该方法（貌似仅在货物新生成时调用），向附近最合适的 *一个或0个* 机器人广播货物信息
// 返回值：是否成功分配
bool Packet::broadcast() {
	queue<pair<int, int>> qu;	// point
	qu.push({x, y});
	vector<vector<int>> vis(GRAPH_SIZE, vector<int>(GRAPH_SIZE, 0));
	vis[x][y] = 1;

	int step = 0;
	bool isok = false;
	
	while (!qu.empty()) {

		int qn = qu.size();
		while (qn--) {
			auto [current_x, current_y] = qu.front(); qu.pop();
			
			// int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
			for (auto &[tx, ty]:dir) {
				int next_x = current_x+tx, next_y = current_y+ty;
				if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0 || vis[next_x][next_y]) {	// 越界或已访问
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
					auto rb_id = 0;
					for (auto &it:robot) {	// 找到对应机器人
						if (it.x == next_x && it.y == next_y) {
							rb_id = it.id;
							break;
						}
					}
					auto &rb = robot[rb_id];
					if (rb.status == 0) {	// 机器人处于恢复状态
						continue;
					}
					if (rb.packet_id == -1 && rb.path.empty()) {	// 没有要取的物品，直接把货物分配给该机器人
						this->status = rb.id;
						rb.target_packet_id = this->id;
						rb.target_berth_id = -1;
						rb.update_dict();	// 更新最短路径
						bool can_arrive = rb.set_and_book_a_path_to(this->x, this->y);	// 设置路径
						
						// !正常不会出现
						if (!can_arrive) {
							fprintf(stderr,"#Warning(Packet::broadcast): [%d]Robot::%d(%d,%d) fail to set path to Packet::%d(%d,%d).\n", frame, rb.id, rb.x, rb.y, this->id, this->x, this->y);
							return false;
						}

						rb.book_get_packet_event(rb.shortest_dict[this->x][this->y]);	// 预定取货事件
						isok = true;
						break;
					} else if (rb.packet_id == -1) {	// 有将要取的物品，已经规划好了路径，判断是否将货物重新分配给他
						auto origin_packet = packet[rb.target_packet_id];
						int val1 = this->value;	// 当前货物价值
						int val2 = origin_packet.value;	// 将要取的货物价值
						int t1 = rb.get_dict_to(this->x, this->y) - frame;	// 机器人到达当前货物所需时间
						int t2 = rb.get_dict_to(origin_packet.x, origin_packet.y) - frame;	// 机器人到原货物所需时间）
						t1 += go_to_which_berth[this->x][this->y].second;	// 当前货物到达泊位所需时间
						t2 += go_to_which_berth[origin_packet.x][origin_packet.y].second;	// 原货物到达泊位所需时间

						auto calc = [](int val, int t) { return double(val)/(t+1); };	// 计算性价比
						if (calc(val1, t1) / calc(val2, t2) >= 1.15) {	// 换货物的性价比大于一定比例
							this->status = rb.id;
							origin_packet.status = -1;
							rb.target_packet_id = this->id;
							rb.target_berth_id = -1;
							rb.cancel_path_book();	// 取消原路径
							rb.update_dict();	// 更新最短路径
							bool can_arrive = rb.set_and_book_a_path_to(this->x, this->y);	// 设置路径
							// !正常不会出现
							if (!can_arrive) {
								fprintf(stderr,"#Warning(Packet::broadcast): [%d]Robot::%d(%d,%d) fail to set path to Packet::%d(%d,%d).\n", frame, rb.id, rb.x, rb.y, this->id, this->x, this->y);
								return false;
							}
							rb.book_get_packet_event(rb.shortest_dict[this->x][this->y]);	// 预定取货事件
							isok = true;
							break;
						}
					}
				}
				vis[next_x][next_y] = 1;
				qu.push({next_x, next_y});
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
	if (!isok) {
		fprintf(stderr,"#Note(Packet::broadcast): [%d]Packet::%d(%d,%d) was not broadcast to any robots.\n", frame, this->id, this->x, this->y);
	}
	return isok;	// 是否成功分配
}


// ---------- end Packet方法实现 ----------


