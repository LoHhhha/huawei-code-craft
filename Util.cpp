#include "Util.hpp"
#include "Param.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"


// ---------- begin init ----------

// 预期复杂度：4e4
// 预处理机器人可到达的点
// 更新robot_can_go
void get_robot_can_go(){
	queue<int>qu;
	for(auto &r:robot){
		qu.push(r.x*GRAPH_SIZE+r.y);
		robot_can_go[r.x][r.y]=true;
	}

	while(!qu.empty()){
		int qn=qu.size();
		while(qn--){
			auto point_hash=qu.front();
			int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
			qu.pop();
			for(auto &[dx,dy]:dir){
				int next_x = current_x+dx, next_y = current_y+dy;
				if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {
					continue;
				}
				if(graph[next_x][next_y]!=-1&&!robot_can_go[next_x][next_y]){
					qu.push(next_x*GRAPH_SIZE+next_y);
					robot_can_go[next_x][next_y]=true;
				}
			}
		}
	}
}

// 预期复杂度：3e7
// 预处理选择泊位可到达的点
// 依赖use_berth
// 更新use_berth_can_go
void get_use_berth_can_go(){
	int full_mask=(1<<BERTH_NUM)-1;
	for(int mask=full_mask;mask>=0;mask=(mask-1)&full_mask){
		queue<int>qu;
		
		auto &berth_can_go=use_berth_can_go[mask];

		for(int i=0;i<GRAPH_SIZE;i++){
			for(int j=0;j<GRAPH_SIZE;j++){
				berth_can_go[i][j]=false;
			}
		}
		if(mask==0){
			break;
		}

		for(int berth_idx=0;berth_idx<BERTH_NUM;berth_idx++){
			if(use_berth){
				for(int i=0;i<BERTH_SIZE;i++){
					for(int j=0;j<BERTH_SIZE;j++){
						qu.push((i+berth[berth_idx].x)*GRAPH_SIZE+j+berth[berth_idx].y);
						berth_can_go[i+berth[berth_idx].x][j+berth[berth_idx].y]=true;
					}
				}
			}
		}

		while(!qu.empty()){
			int qn=qu.size();
			while(qn--){
				auto point_hash=qu.front();
				int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
				qu.pop();
				for(auto &[dx,dy]:dir){
					int next_x = current_x+dx, next_y = current_y+dy;
					if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {
						continue;
					}
					if(graph[next_x][next_y]!=-1&&!berth_can_go[next_x][next_y]){
						qu.push(next_x*GRAPH_SIZE+next_y);
						berth_can_go[next_x][next_y]=true;
					}
				}
			}
		}
	}
}

void get_berth_order(){
	int dict[BERTH_NUM][BERTH_NUM]{0};
	for(int idx=0;idx<BERTH_NUM;idx++){
		int b_x=berth[idx].x,b_y=berth[idx].y;
		queue<int>qu;
		for(int i=0;i<BERTH_NUM;i++){
			for(int j=0;j<BERTH_NUM;j++){
				dict[i][j]=INT_INF;
			}
		}
		for(int i=0;i<BERTH_SIZE;i++){
			int cur_x=b_x+i,cur_y=b_y-1;
			if (cur_x>=GRAPH_SIZE || cur_y>=GRAPH_SIZE || cur_x<0 || cur_y<0) {
				continue;
			}
			if(graph[cur_x][cur_y]==0){
				qu.push(cur_x*GRAPH_SIZE+cur_y);
			}

			cur_y=b_y+BERTH_SIZE;
			if (cur_x>=GRAPH_SIZE || cur_y>=GRAPH_SIZE || cur_x<0 || cur_y<0) {
				continue;
			}
			if(graph[cur_x][cur_y]==0){
				qu.push(cur_x*GRAPH_SIZE+cur_y);
			}

			cur_x=b_x-1,cur_y=b_y+i;
			if (cur_x>=GRAPH_SIZE || cur_y>=GRAPH_SIZE || cur_x<0 || cur_y<0) {
				continue;
			}
			if(graph[cur_x][cur_y]==0){
				qu.push(cur_x*GRAPH_SIZE+cur_y);
			}

			cur_x=b_x+BERTH_SIZE;
			if (cur_x>=GRAPH_SIZE || cur_y>=GRAPH_SIZE || cur_x<0 || cur_y<0) {
				continue;
			}
			if(graph[cur_x][cur_y]==0){
				qu.push(cur_x*GRAPH_SIZE+cur_y);
			}
		}
		int current_dict=0;
		while(!qu.empty()){
			int qn=qu.size();
			while(qn--){
				auto point_hash = qu.front();
				qu.pop();
				int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
				for(auto &[dx,dy]:dir){
					int next_x = current_x+dx, next_y = current_y+dy;
					if (next_x>=b_x+BERTH_NUM || next_y>=b_y+BERTH_NUM || next_x<b_x || next_y<b_y) {
						continue;
					}
					if(dict[next_x-b_x][next_y-b_y]==INT_INF){
						qu.push(next_x*GRAPH_SIZE+next_y);
						dict[next_x-b_x][next_y-b_y]=current_dict;
					}
				}
			}
			current_dict++;
		}
		berth_block_order[idx].resize(BERTH_SIZE*BERTH_SIZE);
		iota(berth_block_order[idx].begin(),berth_block_order[idx].end(),0);
		sort(berth_block_order[idx].begin(),berth_block_order[idx].end(),[&](int &n1,int &n2){
			int x1=n1/BERTH_SIZE,x2=n2/BERTH_SIZE;
			int y1=n1%BERTH_SIZE,y2=n2%BERTH_SIZE;
			return dict[x1][y1]>dict[x2][y2];
		});
		berth_block_order[idx].erase(berth_block_order[idx].begin(),berth_block_order[idx].begin()+NOT_USE_BERTH_BLOCK_NUM);
	}
}

// 期望复杂度：1e7(4e4*(BERTH_NUM choose num=10C5=210)) 
// 选择num个泊位、预处理每一点到最近港口及其距离、绑定船及添加消息
// 港口选择优先级：泊位可到达点数目多优先(首先机器人可达)、距离短者优先、泊位到虚拟点、泊位速度快者优先
// 注意：调用get_robot_can_go、get_use_berth_can_go
void choose_best_berth(int num){
	get_robot_can_go();

	int berths_to_point_dict[GRAPH_SIZE][GRAPH_SIZE][BERTH_NUM]{0};

	// 维护出每一个泊位到【机器人可到达的点】的最短距离
	for (int idx=0;idx<BERTH_NUM;idx++) {
		for(int i=0;i<GRAPH_SIZE;i++){
			for(int j=0;j<GRAPH_SIZE;j++){
				berths_to_point_dict[i][j][idx] = INT_INF;
			}
		}
		int x = berth[idx].x, y = berth[idx].y;

		queue<int> qu;
		qu.push((x)*GRAPH_SIZE+(y));
		qu.push((x)*GRAPH_SIZE+(y+3));
		qu.push((x+3)*GRAPH_SIZE+(y));
		qu.push((x+3)*GRAPH_SIZE+(y+3));

		berths_to_point_dict[x][y][idx] = 0;
		berths_to_point_dict[x+3][y][idx] = 0;
		berths_to_point_dict[x][y+3][idx] = 0;
		berths_to_point_dict[x+3][y+3][idx] = 0;

		int current_dict=1;
		while (!qu.empty()) {
			int qn = qu.size();
			while (qn--) {
				auto point_hash = qu.front();
				qu.pop();
				int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
				for(auto &[dx,dy]:dir){
					int next_x = current_x+dx, next_y = current_y+dy;
					if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {
						continue;
					}
					if(robot_can_go[next_x][next_y]&&berths_to_point_dict[next_x][next_y][idx]==INT_INF){
						qu.push(next_x*GRAPH_SIZE+next_y);
						berths_to_point_dict[next_x][next_y][idx]=current_dict;
					}
				}
			}
			current_dict++;
		}
	}

	bool current_use_berth[BERTH_NUM]{0};
	ll best_reachable_point=0,best_tol_point_dict=LLONG_INF,best_tol_transport_time=LLONG_INF,best_tol_load_time=LLONG_INF;

	// 检查当前子集是否是最优的
	auto check=[&](){
		ll tol_point_dict=0,reachable_point=0,tol_transport_time=0,tol_load_time=0;
		for(int i=0;i<GRAPH_SIZE;i++){
			for(int j=0;j<GRAPH_SIZE;j++){
				if(robot_can_go[i][j]){
					int min_dict_berth_id=-1;
					for(int idx=0;idx<BERTH_NUM;idx++){
						if(current_use_berth[idx]){
							if(min_dict_berth_id==-1||berths_to_point_dict[i][j][idx]<berths_to_point_dict[i][j][min_dict_berth_id]){
								min_dict_berth_id=idx;
							}
						}
					}
					if(min_dict_berth_id!=-1&&berths_to_point_dict[i][j][min_dict_berth_id]!=INT_INF){
						reachable_point++;
					}
					tol_point_dict+=berths_to_point_dict[i][j][min_dict_berth_id];
				}
			}
		}
		for(int idx=0;idx<BERTH_NUM;idx++){
			if(current_use_berth[idx]){
				tol_transport_time+=berth[idx].transport_time;
				tol_load_time+=berth[idx].loading_speed;
			}
		}

		auto swap=[&](){
			best_reachable_point=reachable_point;
			best_tol_point_dict=tol_point_dict;
			best_tol_transport_time=tol_transport_time;
			best_tol_load_time=tol_load_time;
			for(int i=0;i<BERTH_NUM;i++){
				use_berth[i]=current_use_berth[i];
			}
		};

		// 优先级1：能到达更多的点
		if(best_reachable_point<reachable_point){
			swap();
		}
		else if(best_reachable_point==reachable_point){
			// 优先级2：总距离短
			if(best_tol_point_dict>tol_point_dict){
				swap();
			}
			else if(best_tol_point_dict==tol_point_dict){
				// 优先级3：总运输时间短
				if(best_tol_transport_time>tol_transport_time){
					swap();
				}
				else if(best_tol_transport_time==tol_transport_time){
					// 优先级4：装货时间短
					if(best_tol_load_time>tol_load_time){
						swap();
					}
				}
			}
		}
	};
	
	// 枚举BERTH_NUM中选num个泊位的情况，逐个验证是否最优
	function<void(int,int)>choose=[&](int idx,int need){
		if(need==0){
			check();
			return;
		}
		if(idx==BERTH_NUM||BERTH_NUM-idx+1<need){
			return;
		}
		choose(idx+1,need);
		current_use_berth[idx]=true;
		choose(idx+1,need-1);
		current_use_berth[idx]=false;
	};
	choose(0,num);

	// 维护go_to_which_berth
	current_berth_use_hash=0;
	for(int i=0;i<BERTH_NUM;i++){
		if(use_berth[i]){
			current_berth_use_hash|=1<<i;
		}
	}

	int full_mask=(1<<BERTH_NUM)-1;
	for(int mask=full_mask;mask>=0;mask=(mask-1)&full_mask){
		auto &cur_go_to_which_berth=go_to_which_berth[mask];
		for(int i=0;i<GRAPH_SIZE;i++){
			for(int j=0;j<GRAPH_SIZE;j++){
				cur_go_to_which_berth[i][j].first=-1;
				cur_go_to_which_berth[i][j].second=INT_INF;
			}
		}
		if(mask==0)break;

		for(int i=0;i<GRAPH_SIZE;i++){
			for(int j=0;j<GRAPH_SIZE;j++){
				for(int idx=0;idx<BERTH_NUM;idx++){
					if(((mask>>idx)&1)&&berths_to_point_dict[i][j][idx]!=INT_INF){
						if(
							cur_go_to_which_berth[i][j].first==-1||
							cur_go_to_which_berth[i][j].second>berths_to_point_dict[i][j][idx]||(
								cur_go_to_which_berth[i][j].second==berths_to_point_dict[i][j][idx]&&
								berth[idx].transport_time<berth[cur_go_to_which_berth[i][j].first].transport_time
							)
						){
							cur_go_to_which_berth[i][j].first=idx;
							cur_go_to_which_berth[i][j].second=berths_to_point_dict[i][j][idx];
						}
					}
				}
			}
		}
	}


	// 维护所有使用的泊位的位置
	for(int berth_idx=0;berth_idx<BERTH_NUM;berth_idx++){
		if(use_berth[berth_idx]){
			for(int i=0;i<BERTH_SIZE;i++){
				for(int j=0;j<BERTH_SIZE;j++){
					berth_point_hash.insert((i+berth[berth_idx].x)*GRAPH_SIZE+(j+berth[berth_idx].y));
				}
			}
		}
	}

	get_use_berth_can_go();
	get_berth_order();
	
	// debug
	string choose_info;
	for(int i=0;i<BERTH_NUM;i++){
		if(use_berth[i]){
			choose_info+=to_string(i);
			choose_info+=" ";
		}
	}
	if(!choose_info.empty())choose_info.pop_back();
	fprintf(stderr,"#Note(Util::choose_best_berth): choose berth(%s).\n",choose_info.c_str());
}

// ---------- end init ----------


// ---------- begin Packet相关全局函数 ----------

// 生成货物
bool generate_packet(int x, int y, int packet_money) {
	if(use_berth_can_go[current_berth_use_hash][x][y]&&packet_money>=PACKET_VALUE_THRESHOLD){
		Packet p(++packet_id, x, y, packet_money, frame + PACKET_TIME_OUT);	// 在 1000 帧后过期
		packet[packet_id] = p;						// 在货物表中添加
		hash2packet[x*GRAPH_SIZE+y] = packet_id;	// 在哈希表中添加 [[x*GRAPH_SIZE+y] -> packet_id]
		graph[x][y] ^= PACKET_BIT;					// 在图中添加货物标记
		unbooked_packet.insert(packet_id);			// 默认未分配
		msg_handler.add_an_event(frame + PACKET_TIME_OUT-1, packet_id, MSG_PACKET_NEED_DELETE);		// 事件在每帧结束时执行
		return true;
	}
	return false;
}


// 广播货物信息
// void broadcast_packet(int packet_id) {
// 	auto &p = packet[packet_id];	// 在货物表中添加
// 	// 广播货物信息
// 	bool is_assigned = p.broadcast();	// 是否已被分配
// 	if (is_assigned) {
// 		unbooked_packet.erase(packet_id);	// 从未分配货物中删除
// 	}
// }


// 取走货物
// 仅删除 hash2packet 中的记录
void take_packet(int packet_id) {
	auto &p=packet[packet_id];
	graph[p.x][p.y] ^= PACKET_BIT;	// 在图中删除货物标记
	p.status=ROBOT_NUM;		// 不指定谁拿走
	hash2packet.erase(p.x*GRAPH_SIZE+p.y);	// 从哈希表中删除 x*GRAPH_SIZE+y
}

// 删除货物
// 删除 packet 中的记录
void delete_packet(int packet_id) {
	auto it=packet.find(packet_id);
	if(it!=packet.end()){
		hash2packet.erase(it->second.x * GRAPH_SIZE + it->second.y);	// 从哈希表中删除 x*GRAPH_SIZE+y
		packet.erase(it);	// 从货物表中删除
	}
}

// 货物被预定
void packet_be_booked(int packet_id, int robot_id) {
	packet[packet_id].status = robot_id;	// 货物状态：-1：未被预定，其他数字x：已被机器人x预定
	unbooked_packet.erase(packet_id);
	robot[robot_id].target_packet_id=packet_id;
}

// 货物被取消预定
void packet_unbook(int packet_id){
	if(packet.count(packet_id)){
		packet[packet_id].status=-1;
		unbooked_packet.insert(packet_id);
	}
}

// ---------- end Packet相关全局函数 ----------