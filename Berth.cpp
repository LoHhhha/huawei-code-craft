#include "Param.hpp"
#include"Berth.hpp"


// ---------- begin 重载输出流 ----------
ostream& operator<<(ostream& os, const Berth& berth) {
	os << "泊位: 坐标(" << berth.x << ", " << berth.y << "), 运输时间: ";
	os << berth.transport_time << ", 装载速度: " << berth.loading_speed;
	return os;
}
// ---------- end 重载输出流 ----------
