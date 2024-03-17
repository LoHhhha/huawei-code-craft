#pragma once

#include "Param.hpp"

struct Berth {
	int x, y;					// 码头左上角坐标
	int transport_time;			// 运输到虚拟点的时间
	int loading_speed;			// 装载速度: 个/帧
	int current_wait_packet;	// 当前在泊位等待的货物

	friend ostream& operator<<(ostream& os, const Berth& berth);
};



// ---------- begin berth ----------
static bool use_berth[BERTH_NUM]{0};		// 泊位是否被使用
static vector<Berth> berth(BERTH_NUM);	// 码头 vector
// ---------- end berth ----------