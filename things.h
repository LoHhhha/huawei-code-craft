#include<bits/stdc++.h>
using namespace std;

struct Robot {
	int x, y;	// 机器人当前坐标
	int packet_id;	// 0: 无货物 其他整数: 货物编号
	int status;	// 0: 恢复状态 1: 正常运行状态

	friend ostream& operator<<(ostream& os, const Robot& robot);
};

struct Berth {
	int x, y;			// 码头左上角坐标
	int transport_time;	// 运输到虚拟点的时间
	int loading_speed;	// 装载速度: 个/帧

	friend ostream& operator<<(ostream& os, const Berth& berth);
};

struct Packet {
	int id;			// 货物id
	int x, y;		// 货物位置
	int value;		// 货物价值
	int timeout;	// 过期时间：帧

	Packet() = default;
	Packet(int _id, int _x, int _y, int _value, int _timeout) : id(_id), x(_x), y(_y), value(_value), timeout(_timeout) {}
	friend ostream& operator<<(ostream& os, const Packet& packet);
};

struct Boat {
    int status;         // 0：移动中，1：正常装货状态，2：泊位外等待状态
    int berth_id;	    // 目标泊位id，值为-1时表示目标泊位为虚拟点
    int load;           // 目前装载数
    int capacity;  // 船的容量 *初赛固定

	friend ostream& operator<<(ostream& os, const Boat& boat);
};

ostream& operator<<(ostream& os, const Robot& robot) {
	os << "机器人: 坐标(" << robot.x << ", " << robot.y << "), 货物id: ";
	os << robot.packet_id << ", 运行状态: " << robot.status;
	return os;
}

ostream& operator<<(ostream& os, const Berth& berth) {
	os << "泊位: 坐标(" << berth.x << ", " << berth.y << "), 运输时间: ";
	os << berth.transport_time << ", 装载速度: " << berth.loading_speed;
	return os;
}

ostream& operator<<(ostream& os, const Packet& packet) {
	os << "货物: id: " << packet.id << ", 坐标(" << packet.x << ", " << packet.y << "), ";
	os << "价值: " << packet.value << ", 过期帧: " << packet.timeout;
	return os;
}

ostream& operator<<(ostream& os, const Boat& boat) {
	os << "船: 状态: " << boat.status << ", 目标泊位id: " << boat.berth_id << ", ";
	os << "装载数: " << boat.load << ", 容量: " << boat.capacity;
	return os;
}