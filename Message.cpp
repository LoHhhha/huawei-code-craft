#include "Param.hpp"
#include "Util.hpp"
#include "Message.hpp"
#include "Robot.hpp"
#include "Packet.hpp"
#include "Boat.hpp"
#include "Berth.hpp"


// ---------- begin MsgHandler方法实现 ----------

MsgHandler::MsgHandler(){
    // 这里注册事件闭包，闭包类型限定：function<void(Msg)

    // MSG_ROBOT_NEED_GET： 机器人取货
    auto robot_get=[&](Msg msg){
        if(msg.obj_id<0||msg.obj_id>=ROBOT_NUM){
            fprintf(stderr, "#Error: [%d]MsgHandler:: msg(frame:%d, obj_id:%d, event:%d) fail to execute.\n",
                frame, msg.frame, msg.obj_id, msg.event);
            return;
        }
        robot[msg.obj_id].get_packet();
    };
    f[MSG_ROBOT_NEED_GET]=robot_get;

    // MSG_ROBOT_NEED_PULL： 机器人放货
    auto robot_pull=[&](Msg msg){
        if(msg.obj_id<0||msg.obj_id>=ROBOT_NUM){
            fprintf(stderr, "#Error: [%d]MsgHandler:: msg(frame:%d, obj_id:%d, event:%d) fail to execute.\n",
                frame, msg.frame, msg.obj_id, msg.event);
            return;
        }
        robot[msg.obj_id].pull_packet();
    };
    f[MSG_ROBOT_NEED_PULL]=robot_pull;

    // MSG_BOAT_NEED_BACK：船到达虚拟点
    auto boat_back=[&](Msg msg){
        if(msg.obj_id<0||msg.obj_id>=BOAT_NUM){
            fprintf(stderr, "#Error: [%d]MsgHandler:: msg(frame:%d, obj_id:%d, event:%d) fail to execute.\n",
                frame, msg.frame, msg.obj_id, msg.event);
            return;
        }
        boat[msg.obj_id].go_to_berth(boat[msg.obj_id].bind_berth_id);
    };
    f[MSG_BOAT_NEED_BACK]=boat_back;

    // MSG_BOAT_NEED_GO：装载船马上出发
    auto boat_go=[&](Msg msg){
        if(msg.obj_id<0||msg.obj_id>=BOAT_NUM){
            fprintf(stderr, "#Error: [%d]MsgHandler:: msg(frame:%d, obj_id:%d, event:%d) fail to execute.\n",
                frame, msg.frame, msg.obj_id, msg.event);
            return;
        }
        if(boat[msg.obj_id].status==1){
            boat[msg.obj_id].deliver();
        }
    };
    f[MSG_BOAT_NEED_GO]=boat_go;

    // MSG_PACKET_NEED_DELETE：删除货物
    auto packet_delete=[&](Msg msg){
        // 这个要判断有没有被取走，如果被取走了就不用管了
        auto it=packet.find(msg.obj_id);
        if (it==packet.end()||it->second.status>=ROBOT_NUM){
            return;
        }   
        delete_packet(msg.obj_id);
    };
    f[MSG_PACKET_NEED_DELETE]=packet_delete;
}

// 期望复杂度：1
// 判断当前是否有事件发生
bool MsgHandler::event_occurrence_this_frame(){
    while(!pq.empty()&&pq.top().frame<frame){
        auto msg = pq.top();
        pq.pop();
        fprintf(stderr, "#Error: [%d]MsgHandler:: found a out-of-date msg(frame:%d, obj_id:%d, event:%d).\n",
                frame, msg.frame, msg.obj_id, msg.event);
    }
    return !pq.empty()&&pq.top().frame==frame;
}

// 期望复杂度：1
// 检测队列是否有需要完成的事件
// 完成事件返回true，没有事件返回false
bool MsgHandler::check_and_do() {
    if(!event_occurrence_this_frame())return false;
    while(!pq.empty()&&pq.top().frame==frame){
        auto msg = pq.top();
        pq.pop();

        auto f_it=f.find(msg.event);
        if(f_it!=f.end()){
            f_it->second(msg);
        }
        else{
            fprintf(stderr, "#Error: [%d]MsgHandler:: msg(frame:%d, obj_id:%d, event:%d) fail to execute.\n",
                frame, msg.frame, msg.obj_id, msg.event);
        }
    }
    return true;
}

// 期望复杂度：1
// 添加事件
// eg. add_an_event(19528, 2, MSG_ROBOT_NEED_GET) 在帧数19528时，机器人2需要拿货（对象类型由事件推断）
void MsgHandler::add_an_event(int _frame,int _obj_id,int _event){
    pq.push(Msg(_frame,_obj_id,_event));
}

// ---------- end MsgHandler方法实现 ----------
