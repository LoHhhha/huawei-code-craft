#pragma once

#include "util.h"

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

// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Boat& boat) {
	os << "船: 状态: " << boat.status << ", 目标泊位id: " << boat.berth_id << ", ";
	os << "装载数: " << boat.load << ", 容量: " << boat.capacity;
	return os;
}
// ---------- end 重载输出流 ----------

// ---------- begin Boat方法实现 ----------

// 预期复杂度：1
// 指示前往泊位
void Boat::go_to_berth(int target_berth_id){
	printf(SHIP_OP,this->id,target_berth_id);
}

// 预期复杂度：1
// 指示前往交货
void Boat::deliver(){
	printf(GO_OP,this->id);
}
// ---------- end Boat方法实现 ----------