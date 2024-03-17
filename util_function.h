#include "util.h"
#include "Message.h"
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

// 期望复杂度：1e7(4e4*(BERTH_NUM choose num=10C5=210)) 
// 选择num个泊位、预处理每一点到最近港口及其距离、绑定船及添加消息
// 港口选择优先级：泊位可到达点数目多优先(首先机器人可达)、距离短者优先、泊位到虚拟点、泊位速度快者优先
// 注意：依赖【robot_can_go】的值
void choose_best_berth(int num){
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

		int current_dict=0;
		while (!qu.empty()) {
			int qn = qu.size();
			while (qn--) {
				auto point_hash = qu.front();
				qu.pop();
				int current_x = point_hash/GRAPH_SIZE, current_y = point_hash%GRAPH_SIZE;
				berths_to_point_dict[current_x][current_y][idx] = 0;
				for(auto &[dx,dy]:dir){
					int next_x = current_x+dx, next_y = current_y+dy;
					if (next_x>=GRAPH_SIZE || next_y>=GRAPH_SIZE || next_x<0 || next_y<0) {
						continue;
					}
					if(robot_can_go[next_x][next_y]&&berths_to_point_dict[next_x][next_y][idx]!=INT_INF){
						qu.push(next_x*GRAPH_SIZE+next_y);
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
				if(graph[i][j]!=-1){
					int min_dict_berth_id=-1;
					for(int idx=0;idx<BERTH_NUM;idx++){
						if(current_use_berth[idx]){
							if(min_dict_berth_id==-1||berths_to_point_dict[i][j][idx]<berths_to_point_dict[i][j][min_dict_berth_id]){
								min_dict_berth_id=idx;
							}
						}
					}
					if(berths_to_point_dict[i][j][min_dict_berth_id]!=INT_INF){
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
		if(best_reachable_point>reachable_point){
			swap();
		}
		else if(best_reachable_point==reachable_point){
			// 优先级2：能更快到达点
			if(best_tol_point_dict>tol_point_dict){
				swap();
			}
			else if(best_tol_point_dict==tol_point_dict){
				// 优先级3：能更快运货
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
	for(int i=0;i<GRAPH_SIZE;i++){
		for(int j=0;j<GRAPH_SIZE;j++){
			go_to_which_berth[i][j].first=-1;
			go_to_which_berth[i][j].second=INT_INF;
		}
	}
	for(int i=0;i<GRAPH_SIZE;i++){
		for(int j=0;j<GRAPH_SIZE;j++){
			for(int idx=0;idx<BERTH_NUM;idx++){
				if(use_berth[idx]&&berths_to_point_dict[i][j][idx]!=INT_INF){
					if(
						go_to_which_berth[i][j].first==-1||
						go_to_which_berth[i][j].second>berths_to_point_dict[i][j][idx]||(
							go_to_which_berth[i][j].second==berths_to_point_dict[i][j][idx]&&
							berth[idx].loading_speed>berth[go_to_which_berth[i][j].first].loading_speed
						)
					){
						go_to_which_berth[i][j].first=idx;
						go_to_which_berth[i][j].second=berths_to_point_dict[i][j][idx];
					}
				}
			}
		}
	}

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

// ---------- end init ----------