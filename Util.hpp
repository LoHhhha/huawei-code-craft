#pragma once

#include "Param.hpp"


// init

extern void get_robot_can_go();
extern void choose_best_berth(int num);


// packet

extern bool generate_packet(int x, int y, int packet_money);
extern void broadcast_packet(int packet_id);
extern void take_packet(int packet_id);
extern void delete_packet(int packet_id);
extern void packet_be_booked(int packet_id, int robot_id);
extern void packet_unbook(int packet_id);