#include"packet.hpp"
#include"Robot.hpp"
#include"Message.hpp"

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
							fprintf(stderr,"#Error: [%d]Packet::%d(%d,%d) fail to set path to (%d,%d).\n",frame,this->id,this->x,this->y,rb.x,rb.y);
							return false;
						}

						rb.book_get_packet_event(rb.shortest_dict[this->x][this->y]);	// 预定取货事件
						isok = true;
						break;
					} else if (rb.packet_id == -1) {	// 有将要取的物品，已经规划好了路径，判断是否将货物重新分配给他
						auto origin_packet = packet[rb.packet_id];
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
								fprintf(stderr,"#Error: [%d]Packet::%d(%d,%d) fail to set path to (%d,%d).\n",frame,this->id,this->x,this->y,rb.x,rb.y);
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
	return isok;	// 是否成功分配
}


// ---------- end Packet方法实现 ----------


// ---------- begin Packet相关全局函数 ----------

// 生成货物
bool generate_packet(int x, int y, int packet_money) {
	if(robot_can_go[x][y]){
		Packet p(++packet_id, x, y, packet_money, frame + PACKET_TIME_OUT);	// 在 1000 帧后过期
		packet[packet_id] = p;	// 在货物表中添加
		hash2packet[x*GRAPH_SIZE+y] = packet_id;	// 在哈希表中添加 [[x*GRAPH_SIZE+y] -> packet_id]
		graph[x][y] ^= PACKET_BIT;	// 在图中添加货物标记
		msg_handler.add_an_event(frame + PACKET_TIME_OUT-1, packet_id, MSG_PACKET_NEED_DELETE);		// 事件在每帧结束时执行
		return true;
	}
	return false;
}


// 广播货物信息
void broadcast_packet(int packet_id) {
	auto p = packet[packet_id];	// 在货物表中添加
	// 广播货物信息
	bool is_assigned = p.broadcast();	// 是否已被分配
	if (!is_assigned) {
		unbooked_packet.insert(packet_id);	// 添加到未分配货物列表
	}
}


// 取走货物
// 仅删除 hash2packet 中的记录
void take_packet(int packet_id) {
	auto &p=packet[packet_id];
	graph[p.x][p.y] ^= PACKET_BIT;	// 在图中删除货物标记
	hash2packet.erase(p.x*GRAPH_SIZE+p.y);	// 从哈希表中删除 x*GRAPH_SIZE+y
}

// 删除货物
// 删除 packet 中的记录
void delete_packet(int packet_id) {
	auto &p=packet[packet_id];
	hash2packet.erase(p.x * GRAPH_SIZE + p.y);	// 从哈希表中删除 x*GRAPH_SIZE+y
	packet.erase(packet_id);	// 从货物表中删除
}

// 货物被预定
void packet_be_booked(int packet_id, int robot_id) {
	packet[packet_id].status = robot_id;	// 货物状态：-1：未被预定，其他数字x：已被机器人x预定
	unbooked_packet.erase(packet_id);
	robot[robot_id].target_packet_id=packet_id;
}

// 货物被取消预定
void packet_unbook(int packet_id){
	packet[packet_id].status=-1;
	unbooked_packet.insert(packet_id);
}

// ---------- end Packet相关全局函数 ----------