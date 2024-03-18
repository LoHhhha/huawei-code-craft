#pragma once

#include "Param.hpp"

#define SHIP_OP "ship %d %d"	// 船移动指令 ship boat_id[0-4] berth_id[0-9]
#define GO_OP "ship %d"			// 船交货指令 go boat_id[0-4]

struct Boat {
	int id;
    int status;         // 0：移动中，1：正常运行状态(装货/运输完成状态)，2：泊位外等待状态
    int berth_id;	    // 目标泊位id，值为-1时表示目标泊位为虚拟点
	int bind_berth_id;	// 绑定泊位， 注意：船只只会在这个泊位中
    int load;           // 目前装载数
    int capacity;  		// 船的容量 *初赛固定

	Boat() = default;
	Boat(int id, int status, int berth_id, int load, int capacity): id(id), status(status), berth_id(berth_id), load(load), capacity(capacity), bind_berth_id(-1) {}

	void go_to_berth(int target_berth_id);
	void deliver();

	friend ostream& operator<<(ostream& os, const Boat& boat);
};


