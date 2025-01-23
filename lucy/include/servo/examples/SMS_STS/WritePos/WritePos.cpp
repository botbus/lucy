/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "argc error!" << std::endl;
		return 0;
	}
	std::cout << "serial:" << argv[1] << std::endl;
	if (!sm_st.begin(1000000, argv[1]))
	{
		std::cout << "Failed to init sms/sts motor!" << std::endl;
		return 0;
	}
	// sm_st.WritePosEx(1, 1023, 2400, 50);

	int ID{1};
	int16_t P1{682};  // 682
	int16_t P2{1364}; // 1364
	int16_t P3{1023};
	int16_t speed{2400};
	int acc{50};
	// long sleep1 = ();
	// long sleep2 = ();
	sm_st.WritePosEx(ID, P3, speed, acc);
	//	while (1)
	//	{ - P1) / speed) * 1000 + (speed / (acc * 100))
	//		sm_st.WritePosEx(ID, P1, speed, acc); // 舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
	//		usleep(764 * 1000);
	//		usleep(((((P2 - P1) / speed) * 1000) + (speed / (acc * 100)) * 1000) * 1000);
	//		std::cout << "resetting" << "\n";
	//		sm_st.WritePosEx(ID, P2, speed, acc); // 舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置711
	//		usleep(764 * 1000);					  //[(P1-P0)/V]*1000+[V/(A*100)]*1000
	//	}
	sm_st.end();
	return 1;
}
