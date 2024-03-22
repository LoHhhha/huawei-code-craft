#include "Param.hpp"
#include "Boat.hpp"
#include "Berth.hpp"

void send_ship(int id, int berth_id) { cout << "ship " << id << " " << berth_id << endl; };
void send_go(int id) { cout << "go " << id << endl; };

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
	// printf(SHIP_OP,this->id,target_berth_id);
	send_ship(this->id, target_berth_id);
}

// 预期复杂度：1
// 指示前往交货
// 此处同时处理泊位
void Boat::deliver(){
	#ifdef ENABLE_BERTH_DEAD
		if(this->berth_id!=-1){
			auto &b=berth[this->berth_id];
			if(3*b.transport_time+((this->capacity+b.loading_speed-1)/b.loading_speed)/2+frame>FRAME_COUNT){
				current_berth_use_hash&=((1<<BERTH_NUM)-1)^(1<<this->berth_id);
			}
		}
	#endif
	// printf(GO_OP,this->id);
	send_go(this->id);
	trans_packet_count+=this->load;
	this->load=0;
}
// ---------- end Boat方法实现 ----------