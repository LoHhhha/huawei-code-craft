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


bool generate_packet(int x, int y, int packet_money);
void broadcast_packet(int packet_id);
void take_packet(int packet_id);
void delete_packet(int packet_id);
void packet_be_booked(int packet_id, int robot_id);
void packet_unbook(int packet_id);

// ---------- begin packet ----------
static int packet_id = 0;					// 当前最后一个货物id (第一个货物id为1) 
static map<int, Packet> packet;			// 货物id -> 货物信息
static map<int,int> hash2packet;           // point_hash(x*GRAPH_SIZE+y) 转化为货物id
static set<int> unbooked_packet;	        // 未被预定的货物 id
// ---------- end packet ----------
