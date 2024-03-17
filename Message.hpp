#pragma once

#include "Param.hpp"


// 已支持处理事件：
// 与Msg事件序号对应
// 根据事件来完成对obj的分类
// 约定：0-99分配给机器人，100-199分配给船，200-299分配给货物
// 0：机器人到达取货点 -> 取货操作
// 1：机器人到达泊位 -> 放货操作
// 100：船已到达虚拟点 -> 返航
// 101：船出发到虚拟点 -> 出发
// 200：货物过期 -> 删除货物

#define MSG_ROBOT_NEED_GET 0
#define MSG_ROBOT_NEED_PULL 1
#define MSG_BOAT_NEED_BACK 100
#define MSG_BOAT_NEED_GO 101
#define MSG_PACKET_NEED_DELETE 200

struct Robot;
struct Boat;
struct Packet;

struct Msg{
    int frame;      // 事件发生帧数

    int obj_id;

    // 事件序号
    int event;

    bool operator< (const Msg &other) const {
        if(this->frame == other.frame){
            return this->event > other.event;
        }
        return this->frame > other.frame;
    }

public:
    Msg(int frame,int obj_id,int event): frame(frame), obj_id(obj_id), event(event){};
};

class MsgHandler{
private:
    priority_queue<Msg>pq;      // 消息队列
    unordered_map<int,function<void(Msg)>>f;    // 事件闭包
public:
    MsgHandler();
    bool event_occurrence_this_frame();
    bool check_and_do();
    void add_an_event(int _frame,int _obj_id,int _event);
};

// ---------- begin msgHandler ----------
static MsgHandler msg_handler;
// ---------- end msgHandler ----------