#pragma once

#include "Param.hpp"

struct Packet {
	int id;			// 货物id
	int x, y;		// 货物位置
	int value;		// 货物价值
	int timeout;	// 过期时间：帧
	int status;		// 货物状态：-1：未被预定，[0,ROBOT_NUM)机器人预定 [ROBOT_NUM,2*ROBOT_NUM)机器人拿取

	Packet() = default;
	Packet(int _id, int _x, int _y, int _value, int _timeout) : id(_id), x(_x), y(_y), value(_value), timeout(_timeout) {}
	
	bool broadcast();

	friend ostream& operator<<(ostream& os, const Packet& packet);
};


