#include "Mapping.h"

/*
主要作用：将足部末端的xy坐标映射到腿部3508的目标转子角度
*/

double ANG1X;
double ANG2X;
double ANG1M;
double ANG2M;

double
LimitMax_f(double input, double max)
{
	if (input > max)
	{
		input = max;
	}
	else if (input < -max)
	{
		input = -max;
	}
	return input;
}

//  double h = 161.0;

//	double L1 = 88.0;
//	double fT = 4000.0; //2000

double footH = 31.0; // 18.0f
//  double FPI = 3.1415926;
//  double h = 161.0;
double ANG1M, ANG1X;
double ANG2M, ANG2X;

// system_time = xTaskGetTickCount();

double ang1(double x, double y) // 对一个坐标点对称轴对y轴左角（恒正）
{
	double ANG1, L3; //,L2;
					 //	L2 = sqrt(L1*L1 + h*h);
	L3 = sqrt(x * x + (h - y) * (h - y));
	ANG1 = (acos(LimitMax_f((h - y) / L3, 1.0))) * 180.0 / FPI;
	return ANG1;
}

double ang2(double x, double y) // 对一个坐标点对称轴对y轴右角（恒正）
{
	double ANG2, L3; //,L2;
					 //	L2 = sqrt(L1*L1 + h*h);
	L3 = sqrt(x * x + (h - y) * (h - y));
	ANG2 = (acos(LimitMax_f((L1 * L1 + L3 * L3 - L2 * L2) / (2 * L1 * L3), 1.0))) * 180.0 / FPI;
	return ANG2;
}

double Ang1(double x, double y) // 对一个坐标点左电机对y轴角度（恒正）
{
	double Ang1;
	if (x <= 0)
		Ang1 = ang1(x, y) + ang2(x, y);
	else
		Ang1 = ang2(x, y) - ang1(x, y);
	return Ang1;
}

double Ang2(double x, double y) // 对一个坐标点右电机对y轴角度（恒正）
{
	double Ang2;
	if (x > 0)
		Ang2 = ang1(x, y) + ang2(x, y);
	else
		Ang2 = ang2(x, y) - ang1(x, y);
	return Ang2;
}

double ang_x(double ang1, double ang2)
{
	double L3, x;
//	L3 = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos((ang1 + ang2) * FPI / 2.0 / 180.0));
		 L3 = ((2.0 * L1 * cos((ang1 + ang2) * FPI / 2.0 / 180.0)) + sqrt((2 * L1 * cos((ang1 + ang2) * FPI / 2.0 / 180.0)) * (2 * L1 * cos((ang1 + ang2) * FPI / 2.0 / 180.0)) - 4.0 * (L1 * L1 - L2 * L2))) / 2.0;
	if (ang1 >= ang2)
	{
		x = -L3 * sin((ang1 - ang2) * FPI / 2.0 / 180.0);
	}
	else
		x = L3 * sin((ang2 - ang1) * FPI / 2.0 / 180.0);

	return x;
}

double ang_y(double ang1, double ang2)
{
	double L3, y;
	//	L3 = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos((ang1 + ang2) * FPI / 2.0 / 180.0));
	L3 = ((2 * L1 * cos((ang1 + ang2) * FPI / 2.0 / 180.0)) + sqrt((2 * L1 * cos((ang1 + ang2) * FPI / 2.0 / 180.0)) * (2 * L1 * cos((ang1 + ang2) * FPI / 2.0 / 180.0)) - 4.0 * (L1 * L1 - L2 * L2))) / 2.0;
	y = h - L3 * cos(fabs((ang1 - ang2) * FPI / 2.0 / 180.0));
	return y;
}
