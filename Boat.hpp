#pragma once

#include "util.h"

struct Boat {
    int status;         // 0：移动中，1：正常运行状态(装货/运输完成状态)，2：泊位外等待状态
    int berth_id;	    // 目标泊位id，值为-1时表示目标泊位为虚拟点
    int load;           // 目前装载数
    int capacity;  		// 船的容量 *初赛固定

	Boat() = default;
	Boat(int status, int berth_id, int load, int capacity): status(status), berth_id(berth_id), load(load), capacity(capacity) {}

	friend ostream& operator<<(ostream& os, const Boat& boat);
};
vector<Boat> boat(BOAT_NUM);	// 船 vector

// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Boat& boat) {
	os << "船: 状态: " << boat.status << ", 目标泊位id: " << boat.berth_id << ", ";
	os << "装载数: " << boat.load << ", 容量: " << boat.capacity;
	return os;
}
// ---------- end 重载输出流 ----------