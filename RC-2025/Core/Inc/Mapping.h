#ifndef __MAPPING_H
#define __MAPPING_H

#ifdef __cplusplus
extern "C"
{
#endif

// #include "main.h"
#include "struct_typedef.h"
#include "math.h"
#include "Behavior.h"
// #define footH 34.0  //31  38  *37  35  68.0
#define FPI 3.1415926
	// #define h 187.08  //161

#define L1 125.0
#define L2 225.0

#define fT 2000.0  // 1500 2000
#define fT2 2000.0 // 1500 2000
#define fT3 1500.0 // 1500
#define fT4 2000.0 // 1500 2000
#define fT5 2000.0 // 1500 2000
#define fT6 2000.0 // 1500 2000
#define fT7 3000.0 // 1300
#define fT8 4000.0 // 1500  3000

#define angw 30.0

	extern double h;
	extern double footH;

	//	 extern double L1,fT;
	extern double ANG1M, ANG1X;
	extern double ANG2M, ANG2X;

	extern uint8_t flag_angle;
	//		 extern float speed_add;
	extern float speed_add_x;

	extern float speed_add;

	extern double ang1(double x, double y);
	extern double ang2(double x, double y);
	extern double Ang1(double x, double y);
	extern double Ang2(double x, double y);
	extern double Ang11(float t);
	extern double Ang22(float t);
	extern double foot_x(float t);
	extern double foot_y(float t);
	extern double Ang_line1(double x1, double y1, double x2, double y2, double t, uint16_t T);
	extern double Ang_line2(double x1, double y1, double x2, double y2, double t, uint16_t T);
	double ang_x(double ang1, double ang2);
	double ang_y(double ang1, double ang2);

	extern double ANG1X;
	extern double ANG2X;
	extern double ANG1M;
	extern double ANG2M;
#ifdef __cplusplus
}
#endif

#endif /* __dma_H */
