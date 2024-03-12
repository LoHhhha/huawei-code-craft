
struct Robot {
	int x, y;	// 机器人当前坐标
	int packet_id;	// 0: 无货物 其他整数: 货物编号
	int status;	// 0: 恢复状态 1: 正常运行状态
};

struct Berth {
	int x, y;			// 码头左上角坐标
	int transport_time;	// 运输到虚拟点的时间
	int loading_speed;	// 装载速度: 个/帧
};

struct Packet {
	int id;			// 货物id
	int x,y;		// 货物位置
	int value;		// 货物价值
	int timeout;	// 过期时间：帧
};

struct Boat {
    int id;			    // 船id
    int status;         // 0：移动中，1：正常装货状态，2：泊位外等待状态
    int berth_id;	    // 泊位id：值为-1时位于虚拟点，移动时无效
    int load;           // 目前装载数
    int capacity;  // 船的容量 *初赛固定
};
