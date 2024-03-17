#pragma once

#include "util.h"

struct Berth {
	int x, y;					// 码头左上角坐标
	int transport_time;			// 运输到虚拟点的时间
	int loading_speed;			// 装载速度: 个/帧
	int current_wait_packet;	// 当前在泊位等待的货物

	friend ostream& operator<<(ostream& os, const Berth& berth);
};


// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Berth& berth) {
	os << "泊位: 坐标(" << berth.x << ", " << berth.y << "), 运输时间: ";
	os << berth.transport_time << ", 装载速度: " << berth.loading_speed;
	return os;
}
// ---------- end 重载输出流 ----------