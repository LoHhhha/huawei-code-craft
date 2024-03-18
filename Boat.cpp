#include "Param.hpp"
#include"Boat.hpp"


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