#pragma once


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
	int shortest_dict[GRAPH_SIZE][GRAPH_SIZE];		// 最短路记录矩阵，与update_dict / get_dict_to / get_and_book_a_path_to一并使用
	// int sleep[GRAPH_SIZE][GRAPH_SIZE];			// 最短路等待记录矩阵，描述的是【去】该格等待的时间，与update_dict / get_dict_to / get_and_book_a_path_to一并使用
	vector<pii> path;								// 维护机器人路径{frame_to_go（出发时间）, point_hash}，go_to_next_point

	Robot() = default;
	Robot(int id, int x, int y, int status): id(id), x(x), y(y), status(status), target_berth_id(-1), target_packet_id(-1) {
		path.reserve(1000);			// 预留空间，减少内存申请
	}

	void update_dict();
	int get_dict_to(int tx,int ty);
	bool set_and_book_a_path_to(int tx,int ty);
	void cancel_path_book();
	void book_get_packet_event(int arrive_frame);
	void book_pull_packet_event(int arrive_frame);
	int arrive_time();

	// ----- begin high level -----

	bool go_to_next_point();
	bool go_to_nearest_berth();
	bool find_a_best_packet();
	void recover();

	// ----- end high level -----

	void get_packet();
	void pull_packet();

	friend ostream& operator<<(ostream& os, const Robot& robot);
};



