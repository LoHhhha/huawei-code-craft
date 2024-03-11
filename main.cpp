// #include<bits/stdc++.h>
#include<iostream>
#include<vector>
using namespace std;

using ll = long long;
using pii = pair<int, int>;

// #define int ll
#define endl "\n"	// 是否保留待定
#define OPTIO std::ios::sync_with_stdio(0), std::cin.tie(0), std::cout.tie(0)
#define INF 0x3f3f3f3f3f3f3f3f
#define inf 0x3f3f3f3f

#define DE_BUG
#ifdef DE_BUG
    #include "DEBUG.h"
    using namespace DEBUG_;
#endif


struct Robot {
	int x, y;	// 机器人当前坐标
	int goods;	// 0: 无货物 其他整数: 货物编号
	int status;	// 0: 恢复状态 1: 正常运行状态
};
vector<Robot> robot(10);	// 机器人 vector


struct Berth {
	int x, y;			// 码头左上角坐标
	int transport_time;	// 运输到虚拟点的时间
	int loading_speed;	// 装载速度: 个/帧
};
vector<Berth> berth(10);	// 码头 vector


void init() {

}


int main() {
	#ifdef DE_BUG
		// 用 cerr 输出调试信息
		OUTPUT = &std::cerr;
		debug(1)
	#endif

	OPTIO;
	init();

	return 0;
}

