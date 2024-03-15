#include "util.h"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"

// ---------- begin init ----------
void choose_best_berth(int num){
	int berths_to_point_dict[GRAPH_SIZE][GRAPH_SIZE][BERTH_NUM]{0};

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
					if(graph[next_x][next_y]!=-1&&berths_to_point_dict[next_x][next_y][idx]!=INT_INF){
						qu.push(next_x*GRAPH_SIZE+next_y);
					}
				}
			}
			current_dict++;
		}
	}

	bool current_use_berth[BERTH_NUM]{0};
	ll best_reachable_point=0,best_tol_point_dict=LLONG_INF;
	auto check=[&](){
		ll tol_dict=0,reachable_point=0;
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
					tol_dict+=berths_to_point_dict[i][j][min_dict_berth_id];
				}
			}
		}
		if(best_reachable_point<reachable_point){
			best_reachable_point=reachable_point;
			best_tol_point_dict=tol_dict;
		}
	};

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
}

// ---------- end init ----------