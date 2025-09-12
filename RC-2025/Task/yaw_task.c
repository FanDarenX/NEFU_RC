/*
																 _                                _
	 /\                                                         (_)                              (_)
	/  \     _ __    ___   ___   ______    ___   _ __     __ _   _   _ __     ___    ___   _ __   _   _ __     __ _
   / /\ \   | '__|  / _ \ / __| |______|  / _ \ | '_ \   / _` | | | | '_ \   / _ \  / _ \ | '__| | | | '_ \   / _` |
  / ____ \  | |    |  __/ \__ \          |  __/ | | | | | (_| | | | | | | | |  __/ |  __/ | |    | | | | | | | (_| |
 /_/    \_\ |_|     \___| |___/           \___| |_| |_|  \__, | |_| |_| |_|  \___|  \___| |_|    |_| |_| |_|  \__, |
														  __/ |                                                __/ |
														 |___/                                                |___/
*/

#include "yaw_task.h"
#include "keyscan_task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "custom.h"
#include "referee_usart_task.h"
#include "referee.h"
#include "math.h"

TD Cup_pitch_td;
TD Cup_roll_td;
TD yaw_td;
TD pic_td;
TD Uplift_td;
TD Y_td;
TD X_td;

TD Dog_td;
TD Dog_lul_td;
TD Dog_rul_td;
TD Dog_ldl_td;
TD Dog_rdl_td;

TD Dog_lur_td;
TD Dog_rur_td;
TD Dog_ldr_td;
TD Dog_rdr_td;

float Dog_R = 20000.0f;
float Dog_h1 = 0.5f;
float Dog_h2 = 0.55f; // TODO  0.5��һ�ξ���

float Dog_R_lul = 9999999.0f;
float Dog_R_lur = 9999999.0f;
float Dog_R_rul = 9999999.0f;
float Dog_R_rur = 9999999.0f;
float Dog_R_ldl = 9999999.0f;
float Dog_R_ldr = 9999999.0f;
float Dog_R_rdl = 9999999.0f;
float Dog_R_rdr = 9999999.0f;

void TD_init(TD *TT, float r, float h1, float h2)
{
	TT->xx1 = 0; // ��ʼ�� xx1
	TT->xx2 = 0; // ��ʼ�� xx2
	TT->fh1 = 0; // ��ʼ�� fh1
	TT->r = r;
	TT->h1 = h1;
	TT->h2 = h2;
}

/*�źź���Sign*/
int16_t Sign(float x)
{
	if (x > 1E-6f) // 1*10��-6��
		return 1;
	else if (x < 1E-6f)
		return -1;
	else
		return 0;
}

/*�����Ժ��������亯��Fsg*/
int16_t Fsg(float x, float d)
{
	float output = 0;
	output = (Sign(x + d) - Sign(x - d)) / 2;
	return output;
}

/*���ٿ����ۺ����ź���*/
float Fhan(float x1, float x2, float r, float h)
{
	float d, a, a0, a1, a2, y;

	d = r * h * h;
	if (fabs(d) < 1E-6f)
	{
		d = 1E-6f; // ���������
	}

	a0 = h * x2;

	y = x1 + a0;

	if (fabs(y) < 1E-6f)
	{
		y = 1E-6f; // ���� y �ӽ���ʱ�������ȶ���
	}

	a1 = sqrt(d * (d + 8.0f * fabs(y))); // ʹ�� fabs ȷ���Ǹ�

	a2 = a0 + Sign(y) * (a1 - d) * 0.5f;

	a = (a0 + y) * Fsg(y, d) + a2 * (1 - Fsg(y, d));

	if (fabs(a) < 1E-6f)
	{
		a = 1E-6f; // ���� a �ӽ���ʱ�������ȶ���
	}

	return (-r) * (a / d) * Fsg(a, d) - r * Sign(a) * (1 - Fsg(a, d));
}

/*
���� r���ò��������˸���΢���������档
�ϴ�� r ֵ����ʹϵͳ�������Ӧ�仯���������ֵ���ܵ���ϵͳ���ȶ�������񵴡�
ͨ������£�����ѡ��һ����Խ�С��ֵ��ʼ���ԣ����� r = 5 ���� r = 10��Ȼ�����ʵ��Ч��������

���� h1������������ڲ������еķ��������йأ�Ӱ����ϵͳ�Ķ�̬���ԡ�
һ����ԣ�h1 Ӧ��С�ڵ��� 1�����ҿ��Ը��ݾ���Ӧ�ó������е�����
���ڽǶȷ�Χ�� 0 �� 180 ����������Գ��Դ� h1 = 0.5 ��ʼ���۲�����ֲ��ʵ�������

���� h2������ʱ�䲽������˵�ǲ������ڵı������ӣ���ֱ��Ӱ�쵽״̬���µ��ٶȡ�
�������ϵͳ���Խϸߵ�Ƶ�����У��� 1kHz������ô����ѡ���С��ʱ�䲽���������ӣ�
��֮����ѡ��ϴ��ֵ�����ǵ��Ƕȵı仯�ʲ����ر�ߣ����Դ� h2 = 0.01 ���� h2 = 0.005 ��ʼ���ԡ�
*/
void Tracking_Differentiator(TD *TT, float target, float r, float h1, float h2)
{
	if (isnan(target))
	{
		// ���� target Ϊ NaN �����
		target = 0; // ��������һ�������Ĭ��ֵ
	}
	// ���� Fhan ����ʱȷ�� d ��Ϊ��
	float d = r * h1 * h1;
	if (fabs(d) < 1E-6f)
	{
		d = 1E-6f;
	}

	TT->fh1 = Fhan(TT->xx1 - target, TT->xx2, r, h1);

	// ����״̬����
	TT->xx1 = TT->xx1 + h2 * TT->xx2;
	TT->xx2 = TT->xx2 + h2 * TT->fh1;
}

void TD_Fast_Change(float Dog_R)
{
	Dog_R_lul = Dog_R;
	Dog_R_lur = Dog_R;
	Dog_R_rul = Dog_R;
	Dog_R_rur = Dog_R;
	Dog_R_ldl = Dog_R;
	Dog_R_ldr = Dog_R;
	Dog_R_rdl = Dog_R;
	Dog_R_rdr = Dog_R;
}

void TD_INIT(void)
{
	/*ң����*/ // ���� r ��С����Ӧ�ٶ������
	TD_init(&Cup_pitch_td, 100000.0f, 0.5f, 0.005f);
	TD_init(&Cup_roll_td, 1000000.0f, 0.5f, 0.005f);
	TD_init(&yaw_td, 0.6f, 0.5f, 0.005f); //! ���ú��޸�,ƥ��˳��          //test:�ӿ�С���ٶȣ�ͬʱ����yaw : pic = 1 ��1.5(ԭ��0.3��0.45)
	TD_init(&pic_td, 0.9f, 0.5f, 0.005f);
	TD_init(&Uplift_td, 20000.0f, 0.5f, 0.005f); // 8000.0/15000.0f//13000.0f//900000000.0f
	TD_init(&Y_td, 120000.0f, 0.5f, 0.005f);	 // 36000.0f
	TD_init(&X_td, 250000.0f, 0.5f, 0.005f);	 // 50000��Ӧ�м�̧��߼�������

	TD_init(&Dog_td, 20000.0f, 0.5f, 0.005f); // 8000.0/15000.0f//13000.0f//900000000.0f
	TD_init(&Dog_lul_td, Dog_R_lul, Dog_h1, Dog_h2);
	TD_init(&Dog_rul_td, Dog_R_lur, Dog_h1, Dog_h2);
	TD_init(&Dog_ldl_td, Dog_R_rul, Dog_h1, Dog_h2);
	TD_init(&Dog_rdl_td, Dog_R_rur, Dog_h1, Dog_h2);

	TD_init(&Dog_lur_td, Dog_R_ldl, Dog_h1, Dog_h2);
	TD_init(&Dog_rur_td, Dog_R_ldr, Dog_h1, Dog_h2);
	TD_init(&Dog_ldr_td, Dog_R_rdl, Dog_h1, Dog_h2);
	TD_init(&Dog_rdr_td, Dog_R_rdr, Dog_h1, Dog_h2);
}

void TD_task(void)
{
	Tracking_Differentiator(&Dog_lul_td, (Dog_target_lul + Read_Cpa[lul]) / 360.0f * 19.0f * 8192.0f, Dog_lul_td.r, Dog_lul_td.h1, Dog_lul_td.h2);
	Tracking_Differentiator(&Dog_rul_td, (Dog_target_rul + Read_Cpa[rul]) / 360.0f * 19.0f * 8192.0f, Dog_rul_td.r, Dog_rul_td.h1, Dog_rul_td.h2);
	Tracking_Differentiator(&Dog_ldl_td, (Dog_target_ldl + Read_Cpa[ldl]) / 360.0f * 19.0f * 8192.0f, Dog_ldl_td.r, Dog_ldl_td.h1, Dog_ldl_td.h2);
	Tracking_Differentiator(&Dog_rdl_td, (Dog_target_rdl + Read_Cpa[rdl]) / 360.0f * 19.0f * 8192.0f, Dog_rdl_td.r, Dog_rdl_td.h1, Dog_rdl_td.h2);
	Tracking_Differentiator(&Dog_lur_td, (Dog_target_lur + Read_Cpa[lur]) / 360.0f * 19.0f * 8192.0f, Dog_lur_td.r, Dog_lur_td.h1, Dog_lur_td.h2);
	Tracking_Differentiator(&Dog_rur_td, (Dog_target_rur + Read_Cpa[rur]) / 360.0f * 19.0f * 8192.0f, Dog_rur_td.r, Dog_rur_td.h1, Dog_rur_td.h2);
	Tracking_Differentiator(&Dog_ldr_td, (Dog_target_ldr + Read_Cpa[ldr]) / 360.0f * 19.0f * 8192.0f, Dog_ldr_td.r, Dog_ldr_td.h1, Dog_ldr_td.h2);
	Tracking_Differentiator(&Dog_rdr_td, (Dog_target_rdr + Read_Cpa[rdr]) / 360.0f * 19.0f * 8192.0f, Dog_rdr_td.r, Dog_rdr_td.h1, Dog_rdr_td.h2);
	/*-TD-*/
	// if (remoteControl)
	// {
	// 	Tracking_Differentiator(&Cup_pitch_td, Cup_target_pitch, Cup_pitch_td.r, Cup_pitch_td.h1, Cup_pitch_td.h2);
	// 	Tracking_Differentiator(&Cup_roll_td, Cup_target_roll, Cup_roll_td.r, Cup_roll_td.h1, Cup_roll_td.h2);
	// 	Tracking_Differentiator(&yaw_td, yaw, yaw_td.r, yaw_td.h1, yaw_td.h2);
	// 	Tracking_Differentiator(&pic_td, pic, pic_td.r, pic_td.h1, pic_td.h2);
	// 	Tracking_Differentiator(&Uplift_td, Z_target, Uplift_td.r, Uplift_td.h1, Uplift_td.h2);
	// 	Tracking_Differentiator(&Y_td, Y_target, Y_td.r, Y_td.h1, Y_td.h2);
	// 	Tracking_Differentiator(&X_td, X_target, X_td.r, X_td.h1, X_td.h2);

	// 	Tracking_Differentiator(&Dog_td, Dog_target, Dog_td.r, Dog_td.h1, Dog_td.h2);
	// 	}
	// if (keyboard || Custom_action || key_task.custom_mode || !reposition_flag_Y || !reposition_flag_Cup)
	// {
	// 	Tracking_Differentiator(&Cup_pitch_td, Cup_pitch_temp, Cup_pitch_td.r, Cup_pitch_td.h1, Cup_pitch_td.h2);
	// 	Tracking_Differentiator(&Cup_roll_td, Cup_roll_temp, Cup_roll_td.r, Cup_roll_td.h1, Cup_roll_td.h2);
	// 	Tracking_Differentiator(&yaw_td, yaw_temp, yaw_td.r, yaw_td.h1, yaw_td.h2);
	// 	Tracking_Differentiator(&pic_td, pic_temp, pic_td.r, pic_td.h1, pic_td.h2);
	// 	Tracking_Differentiator(&Uplift_td, Z_temp, Uplift_td.r, Uplift_td.h1, Uplift_td.h2);
	// 	Tracking_Differentiator(&Y_td, Y_temp, Y_td.r, Y_td.h1, Y_td.h2);
	// 	Tracking_Differentiator(&X_td, X_temp, X_td.r, X_td.h1, X_td.h2);

	// 	Tracking_Differentiator(&Dog_td, Dog_target, Dog_td.r, Dog_td.h1, Dog_td.h2);
	// 	Tracking_Differentiator(&Dog_lul_td, Dog_target_lul / 360.0f * 19.0f * 8192.0f, Dog_lul_td.r, Dog_lul_td.h1, Dog_lul_td.h2);
	// 	Tracking_Differentiator(&Dog_rul_td, Dog_target_rul / 360.0f * 19.0f * 8192.0f, Dog_rul_td.r, Dog_rul_td.h1, Dog_rul_td.h2);
	// 	Tracking_Differentiator(&Dog_ldl_td, Dog_target_ldl / 360.0f * 19.0f * 8192.0f, Dog_ldl_td.r, Dog_ldl_td.h1, Dog_ldl_td.h2);
	// 	Tracking_Differentiator(&Dog_rdl_td, Dog_target_rdl / 360.0f * 19.0f * 8192.0f, Dog_rdl_td.r, Dog_rdl_td.h1, Dog_rdl_td.h2);
	// 	Tracking_Differentiator(&Dog_lur_td, Dog_target_lur / 360.0f * 19.0f * 8192.0f, Dog_lur_td.r, Dog_lur_td.h1, Dog_lur_td.h2);
	// 	Tracking_Differentiator(&Dog_rur_td, Dog_target_rur / 360.0f * 19.0f * 8192.0f, Dog_rur_td.r, Dog_rur_td.h1, Dog_rur_td.h2);
	// 	Tracking_Differentiator(&Dog_ldr_td, Dog_target_ldr / 360.0f * 19.0f * 8192.0f, Dog_ldr_td.r, Dog_ldr_td.h1, Dog_ldr_td.h2);
	// 	Tracking_Differentiator(&Dog_rdr_td, Dog_target_rdr / 360.0f * 19.0f * 8192.0f, Dog_rdr_td.r, Dog_rdr_td.h1, Dog_rdr_td.h2);
	// }
}
