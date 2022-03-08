/**
  ****************************辽宁科技大学COD****************************
  * @file       user_task.c/h-雨落长安
  * @brief      一个普通程序，请大家友善对待，谢谢大家，写的demo，未来得及封装，凑合看吧
  ==============================================================================
  @endverbatim
  ****************************辽宁科技大学COD****************************
  */

#include "ui_task.h"
#include "referee.h"
#include "main.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_Receive.h"//加载摩擦轮发射闭环真实转速
#define PI 3.1415926
extern uint8_t Robot_ID;//当前机器人的ID
extern uint16_t Cilent_ID;//发送者机器人对应的客户端ID

extern uint8_t Judge_Self_ID;//当前机器人的ID
extern uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11;
String_Data CH_SHOOT;
String_Data CH_FLRB;
String_Data CH_SUPERCAP;
String_Data CH_MODE;
String_Data CH_AUTO;
String_Data CH_MAG;
Float_Data FLOAT_SUPER;
char shoot_arr[5]="SHOOT";
char flrb_arr[4]="FRBL";
char super_arr[6]="SUPER:";
char mode_arr[6] = "NORMOL";
char auto_arr[4] = "AUTO";
char	mag_arr[7] = "MAGZINE";
void ui_task(void *pvParameters);

void ui_task(void *pvParameters)
{
	
		vTaskDelay(1000);
		determine_ID();
		Robot_ID = Judge_Self_ID;
		Cilent_ID = Judge_SelfClient_ID;
	
		memset(&G1,0,sizeof(G1));//中心垂线
		memset(&G2,0,sizeof(G2));//上击打线
		memset(&G3,0,sizeof(G3));//中心水平线
		memset(&G4,0,sizeof(G4));//枪管轴心线
		memset(&G5,0,sizeof(G5));//下击打线
		memset(&G6,0,sizeof(G6));//远距离击打线
		memset(&G7,0,sizeof(G7));//摩擦轮状态
		memset(&G8,0,sizeof(G8));//中央瞄准圈
		memset(&G9,0,sizeof(G9));//自瞄状态
		memset(&G10,0,sizeof(G10));//自瞄状态
		memset(&FLOAT_SUPER,0,sizeof(FLOAT_SUPER));//超级电容电量百分比浮点数
		memset(&CH_SHOOT,0,sizeof(CH_SHOOT));//摩擦轮标识
		memset(&CH_SUPERCAP,0,sizeof(CH_SUPERCAP));//超级电容标识
		memset(&CH_MODE,0,sizeof(CH_MODE));//模式标识
		memset(&CH_MAG,0,sizeof(CH_MAG));//模式标识

		static double angle =0;
		/*绘制中心瞄准线*/
		Line_Draw(&G1,"001",UI_Graph_ADD,9,UI_Color_Cyan,1,960,330,960,620);
		Line_Draw(&G2,"002",UI_Graph_ADD,9,UI_Color_Cyan,1,880,580,1040,580);
		Line_Draw(&G3,"003",UI_Graph_ADD,9,UI_Color_Cyan,1,800,540,1120,540);
		Line_Draw(&G4,"004",UI_Graph_ADD,9,UI_Color_Cyan,1,880,500,1040,500);
		Line_Draw(&G5,"005",UI_Graph_ADD,9,UI_Color_Cyan,1,900,420,1020,420);
		Line_Draw(&G6,"006",UI_Graph_ADD,9,UI_Color_Cyan,1,920,370,1000,370);
		
		/*绘制中央瞄准圈*/
		Circle_Draw(&G8,"007",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
	
		/*绘制当前模式字符*/
		Char_Draw(&CH_MODE,"008",UI_Graph_ADD,9 ,UI_Color_Yellow,20,6,4,80,840,&mode_arr[0]);
		
		/*绘制SUPER字符*/
		Char_Draw(&CH_SUPERCAP,"009",UI_Graph_ADD,9 ,UI_Color_Yellow,20,6,4,80,790,&super_arr[0]);
		/*绘制超级电容剩余电量*/
		Float_Draw(&FLOAT_SUPER,"010",UI_Graph_ADD,9,UI_Color_Yellow,20,1,4,170,790,16.6);
		
		/*绘制SHOOT字符*/
		Char_Draw(&CH_SHOOT,"011",UI_Graph_ADD,9 ,UI_Color_Yellow,20,5,4,80,740,&shoot_arr[0]);
		/*绘制SHOOT指示灯(摩擦轮状态)*/
		Circle_Draw(&G7,"012",UI_Graph_ADD,9,UI_Color_Yellow,15,230,730,10); 
		
		/*绘制当前自瞄情况字符*/
		Char_Draw(&CH_AUTO,"013",UI_Graph_ADD,9 ,UI_Color_Yellow,20,4,4,80,690,&auto_arr[0]);
		/*绘制自瞄情况指示灯*/
		Circle_Draw(&G9,"014",UI_Graph_ADD,9,UI_Color_Yellow,15,230,680,10);  

		/*绘制弹仓情况字符*/
		Char_Draw(&CH_MAG,"015",UI_Graph_ADD,9 ,UI_Color_Yellow,20,7,4,80,640,&mag_arr[0]);
		/*绘制弹仓情况指示灯*/
		Circle_Draw(&G10,"016",UI_Graph_ADD,9,UI_Color_Yellow,15,250,630,10);  

  while(1)
	{
		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);
		UI_ReFresh(1,G8);
		
		Char_ReFresh(CH_MODE);
		
		Char_ReFresh(CH_SUPERCAP);
		UI_ReFresh(1,FLOAT_SUPER);
		
		Char_ReFresh(CH_SHOOT);
		
		Char_ReFresh(CH_AUTO);
		UI_ReFresh(1,G9);
		
		Char_ReFresh(CH_MAG);
		UI_ReFresh(1,G10);

		//频率控制10hz
		vTaskDelay(100);
	}		
}




