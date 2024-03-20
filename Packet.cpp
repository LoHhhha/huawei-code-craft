#include "Param.hpp"
#include "Util.hpp"
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

// 最坏时间复杂度：O(4e4(bfs) + 1e6(update_dict) + 1e5(set_and_book_a_path_to))
// 每当有货物生成时调用该方法（貌似仅在货物新生成时调用），向附近最合适的 *一个或0个* 机器人广播货物信息
// 返回值：是否成功分配
bool broadcast_vis[GRAPH_SIZE][GRAPH_SIZE];
bool Packet::broadcast() {
	queue<pair<int, int>> qu;	// point
	qu.push({x, y});
	memset(broadcast_vis, false, sizeof(broadcast_vis));
	broadcast_vis[x][y] = true;

	int step = 0;
	bool isok = false;
	
	while (!qu.empty()) {

		int qn = qu.size();
		while (qn--) {
			auto [current_x, current_y] = qu.front(); qu.pop();
			
			for (auto &[tx, ty]:dir) {
				int next_x = current_x+tx, next_y = current_y+ty;
				if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0 || broadcast_vis[next_x][next_y]) {	// 越界或已访问
					continue;
				}
				if (graph[next_x][next_y] == -1) {	// 障碍
					continue;
				}
				if (step+1 >= PACKET_TIME_OUT-ARRIVE_PACKET_OFFSET) {	// 超时，剩余200帧机动
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

					// 没有要取的物品，直接把货物分配给该机器人
					// 这里修改为target_packet_id，path为空可能是到达货物了
					if (rb.packet_id == -1 && rb.target_packet_id==-1) {
						rb.update_dict();	// 更新最短路径
						bool can_arrive = rb.set_and_book_a_path_to(this->x, this->y);	// 设置路径
						
						// !正常不会出现
						if (!can_arrive) {
							fprintf(stderr,"#Warning(Packet::broadcast): [%d]Robot::%d(%d,%d) fail to set path to Packet::%d(%d,%d).\n", frame, rb.id, rb.x, rb.y, this->id, this->x, this->y);
							return false;
						}

						rb.target_packet_id = this->id;
						rb.target_berth_id = -1;

						packet_be_booked(this->id,rb.id);		// 预定
						rb.book_get_packet_event(rb.get_dict_to(this->x,this->y));	// 预定取货事件

						isok = true;
						break;
					} else if (rb.packet_id == -1) {	// 有将要取的物品，已经规划好了路径，判断是否将货物重新分配给他
						auto origin_packet = packet[rb.target_packet_id];
						int val1 = this->value;	// 当前货物价值
						int val2 = origin_packet.value;	// 将要取的货物价值

						if(val1<=val2){
							continue;
						}
						// 机器人到达的路径用一下方法求解不正确
						// rb.get_dict_to必须经过update_dict后才是正确
						// int t1 = rb.get_dict_to(this->x, this->y) - frame;	// 机器人到达当前货物所需时间
						// int t2 = rb.get_dict_to(origin_packet.x, origin_packet.y) - frame;	// 机器人到原货物所需时间）
						int t1=step+1;		// 机器人到达当前货物所需时间
						int t2=rb.arrive_time()-frame;

						t1 += go_to_which_berth[this->x][this->y].second;	// 当前货物到达泊位所需时间
						t2 += go_to_which_berth[origin_packet.x][origin_packet.y].second;	// 原货物到达泊位所需时间

						t1 +=frame-rb.pre_pull_packet_frame;

						auto calc = [](int val, int t)->double { return double(val)/(t+1); };	// 计算性价比
						if (calc(val1, t1)>= calc(val2, t2)*PACKET_SWITCH_RATE) {	// 换货物的性价比大于一定比例
							
							// 解除原有货物预定
							packet_unbook(rb.target_packet_id);

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

							packet_be_booked(this->id,rb.id);		// 预定
							rb.book_get_packet_event(rb.get_dict_to(this->x,this->y));	// 预定取货事件

							isok = true;
							break;
						}
					}
				}
				broadcast_vis[next_x][next_y] = true;
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


