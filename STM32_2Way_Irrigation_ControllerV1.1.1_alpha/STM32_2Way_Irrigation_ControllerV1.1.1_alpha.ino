// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_2Way_Irrigation_ControllerV1.1.1_alpha.ino
    Created:	2020/3/4 15:49:47
    Author:     刘家辉
*/

/*
 * 主函数文件。
 * setup部分的功能有：复用调试引脚、开启独立看门狗、设备串口波特率、初始化各类总线
 * 工程模式、注册服务器申请、校正因突发断电的电机开度卷膜等。
 * loop部分的功能有：矫正开度误差、LoRa监听服务器指令、手动卷膜监测、定时自检参数等。
 *
 * 如有任何疑问，请发送邮件到： liujiahiu@qq.com
 */

#include <arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/iwdg.h>
#include <RTClock.h>
#include <wirish_debug.h>

#include "Modbus.h"
#include "ModbusSerial.h"
#include "Set_coil.h"
#include "i2c.h"
#include "Command_Analysis.h"
#include "receipt.h"
#include "AT24CXX.h"
#include "Memory.h"
#include "LoRa.h"
#include "public.h"
#include "Security.h"
#include "fun_periph.h"
#include "Private_Timer.h"
#include "Private_RTC.h"

/* 测试宏 */
#define DEBUG_PRINT 	0 	//打印调试信息
#define Reset_RTC 		0   //重置rtc时间
#define Get_RTC			0	//获取rtc时间
#define Reset_Binding	0	//重新绑定
/* 使能宏 */
#define SOFT_HARD_VERSION	1 //使能写入软件和硬件版本
#define USE_RTC				1 //使用RTC
#define USE_TIMER			0 //使用定时器
#define USE_COM				1 //使用串口发送数据
#define USE_KEY				0 //使用按键发送数据
#define USE_NODE			0 //loRa节点模式
#define USE_GATEWAY			1 //loRa网关模式
/* 替换宏 */
#define Software_version_high 	0x01 	//软件版本的高位
#define Software_version_low 	0x11  	//软件版本的低位
#define Hardware_version_high 	0x01 	//硬件版本的高位
#define Hardware_version_low 	0x00	//硬件版本的低位
#define Init_Area				0x01	//初始区域ID


//全局变量
String comdata = "";//串口接收的字符串
unsigned char gSN_Code[9] = { 0x00 }; //设备出厂默认SN码全为0
unsigned char gRTC_Code[7] = { 20,20,00,00,00,00,00 };//
bool DOStatus_Change = false;//DO的状态改变标志位
bool One_Cycle_Complete = false;//一轮循环完成的标志位
bool Cyclic_intervalFlag = false;//循环时间间隔的标志位
bool Cyclic_timing = false;//正在进行循环时间间隔的标志位
bool DO_intervalFlag = false;//单个间隔时间的标志位
bool DO_interval_timing = false;//正在进行单个间隔时间的标志位
unsigned int KeyDI7_num = 0;//按键DI7按下的次数
unsigned char GET_DO_ON = 0;//开启的数量++


//函数声明
void Request_Access_Network(void);		//检测是否已经注册到服务器成功
void Project_Debug(void);				//工程模式
void Key_Reset_LoRa_Parameter(void);	//按键重置LORA参数
void Irrigation_time_sharing_onV2(void);//灌溉分时开启V2
void Irrigation_time_sharing_onV3(void);//灌溉分时开启V3
void Key_cycle_irrigationV2(void);		//按键启动循环灌溉v2
void Key_cycle_irrigationV3(void);		//按键启动循环灌溉v3
void Regular_status_report(void);		//定时状态上报
void Change_status_report(void);		//状态改变上报
void Com_Set_Cyclic_interval(void);		//串口设置循环间隔
void Judge_Cycle_completion();			//判断灌溉循环是否完成



// the setup function runs once when you press reset or power the board
void setup()
{
	Some_Peripheral.Peripheral_GPIO_Pinmode();//进行引脚的pinmode配置

	Modbus_Coil.Modbus_Config();//添加线圈寄存器

	/*Serial Wire debug only (JTAG-DP disabled, SW-DP enabled)*/
	// C:\Program Files (x86)\Arduino\hardware\stm32\STM32F1\system\libmaple\stm32f1\include\series
	afio_cfg_debug_ports(AFIO_DEBUG_NONE);//设置为JTAG和SW禁用

	/*配置IWDG*/
	iwdg_init(IWDG_PRE_256, 2000); //6.5ms * 1000 = 6500ms.

	Serial.begin(9600); //USART1, 当使用USART下载程序：USART--->USART1
	LoRa_MHL9LF.BaudRate(9600);//#define LoRa_Serial     Serial2
	// Serial2.begin(9600);//485的串口

	bkp_init();	//备份寄存器初始化使能
	EEPROM_Operation.EEPROM_GPIO_Config();		//设置EEPROM读写引脚
	Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
	iwdg_feed();

	LoRa_MHL9LF.LoRa_GPIO_Config();
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
	/*
	 *上电后LoRa模块会发送厂家信息过来
	 *这个时候配置的第一个参数在校验回车换行等参数
	 *的时候会受到影响。必须先接收厂家信息并清空缓存
	 */
	delay(2000);
	gIsHandleMsgFlag = false;
	LoRa_Command_Analysis.Receive_LoRa_Cmd(); //从网关接收LoRa数据
	gIsHandleMsgFlag = true;
	iwdg_feed();

	//Initialize LoRa parameter.
#if USE_GATEWAY
	LoRa_Para_Config.Save_LoRa_Com_Mode(0xF1);//这里是写入模式为网关模式
#else
	LoRa_Para_Config.Save_LoRa_Com_Mode(0xF0);//这里是写入模式为节点模式
#endif
	LoRa_MHL9LF.Parameter_Init(false);
	LoRa_Para_Config.Save_LoRa_Config_Flag();

#if SOFT_HARD_VERSION
	Serial.println("");

	//软件版本存储程序
	if (Software_version_high == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR) &&
		Software_version_low == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("Software_version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	else
	{
		Vertion.Save_Software_version(Software_version_high, Software_version_low);
		Serial.println(String("Successfully store the software version, the current software version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	//硬件版本存储程序
	if (Hardware_version_high == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR) &&
		Hardware_version_low == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("Hardware_version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
	else
	{
		Vertion.Save_hardware_version(Hardware_version_high, Hardware_version_low);
		Serial.println(String("Successfully store the hardware version, the current hardware version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
#endif

#if Reset_RTC
	Private_RTC.Update_RTC(&gRTC_Code[0]);
#endif // Reset_RTC

#if Get_RTC
	/*得到随机值*/
	unsigned char random_1 = random(0, 255);
	unsigned char random_2 = random(0, 255);

	/*这里上报完成一个轮次循环*/
	Message_Receipt.Irrigation_loop_Receipt(false, 1, random_1, random_2);
#endif // Get_RTC

	Project_Debug(); //工程模式

	// SN.Clear_SN_Access_Network_Flag(); //清除注册到服务器标志位

	/*Request access network(request gateway to save the device's SN code and channel)*/
	Request_Access_Network(); //检查是否注册到服务器

	while (SN.Self_check(gSN_Code) == false)
	{
		// LED_SELF_CHECK_ERROR;
		Serial.println("");
		Serial.println("Verify SN code failed, try to Retrieving SN code...");
		//Serial.println("验证SN代码失败，尝试找回SN代码…");
		Message_Receipt.Request_Device_SN_and_Channel(); //当本地SN码丢失，向服务器申请本机的SN码
		LoRa_Command_Analysis.Receive_LoRa_Cmd();		 //接收LORA参数
		MyDelayMs(3000);
		iwdg_feed();
	}
	Serial.println("SN self_check success...");//SN自检成功
	// LED_RUNNING;
	iwdg_feed();

	// Realtime_Status_Reporting_Timer_Init(); //使用定时器2初始化自动上报状态周期(改为了定时器3)
	Stop_Status_Report_Timing();
	Serial.println("Timed status reporting mechanism initialization completed...");//定时上报机制初始化完成
	Serial.println("");

	// Self_Check_Parameter_Timer_Init(); //使用定时器3初始化自检参数功能自检周期
	Stop_Self_Check_Timing();
	Serial.println("Timed self check mechanism initialization completed...");//定时自检机制初始化完成
	Serial.println("");

	Irrigation_Timer_Init();//使用定时器4初始化灌溉计时
	Stop_Timer4();//不用的时候关闭
	Serial.println("Irrigation timing mechanism initialization completed...");//灌溉计时机制初始化完成
	Serial.println("");

	unsigned char random_1 = random(0, 255);unsigned char random_2 = random(0, 255);
	Message_Receipt.OnLine_Receipt(true, 2, random_1, random_2);//设备上线状态报告
	Serial.println("Online status report");
	Serial.println("");

	Serial.println("All configuration items are initialized. Welcome to use.  ~(*^__^*)~ ");//所有的设置项初始化完成，欢迎使用
	Serial.println("");

	// while (1)
	// {
	// 	iwdg_feed();
	// 	// digitalWrite(KCZJ1,LOW);digitalWrite(KCZJ2,HIGH);
	// 	Set_Irrigation_relay(0, ON);
	// 	Set_Irrigation_relay(1, ON);
	// 	Serial.println("ON");
	// 	delay(1000);
	// 	// digitalWrite(KCZJ1,HIGH);digitalWrite(KCZJ2,LOW);
	// 	Set_Irrigation_relay(0, OFF);
	// 	Set_Irrigation_relay(1, OFF);
	// 	Serial.println("OFF");
	// 	delay(2000);
	// }	
}

/*
 @brief     : loop
 @param     : 无
 @return    : 无
 */
void loop()
{
	unsigned char *_empty = NULL;

	iwdg_feed();

	LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据

	Modbus_Coil.Modbus_Realization(_empty, 0);//设置输出线圈状态，modbus实现

	Check_Store_Param_And_LoRa(); //检查存储参数以及LORA参数

	Regular_status_report();//定时状态上报

	Change_status_report();//状态改变上报

	iwdg_feed();
	Irrigation_time_sharing_onV3();//灌溉分时打开

	Key_cycle_irrigationV3();//按键启动循环灌溉

	Com_Set_Cyclic_interval();//串口设置循环间隔
}

/*
 @brief   : 检测是否已经注册到服务器成功，如果没有注册，则配置相关参数为默认参数，然后注册到服务器。
			没有注册成功，红灯1每隔500ms闪烁。
			Checks whether registration with the server was successful, and if not,
			configures the relevant parameters as default parameters and registers with the server.
			Failing registration, red light flashes every 500ms.
 @para    : None
 @return  : None
 */
void Request_Access_Network(void)
{
	if (SN.Verify_SN_Access_Network_Flag() == false)
	{
		gAccessNetworkFlag = false;

		if (SN.Save_SN_Code(gSN_Code) && SN.Save_BKP_SN_Code(gSN_Code))
			Serial.println("Write Inital SN success... <Request_Access_Network>");

		if (SN.Clear_Area_Number() && SN.Clear_Group_Number())
		{
			Serial.println("Already Clear area number and group number... <Request_Access_Network>");
		}

		unsigned char Default_WorkGroup[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
		if (SN.Save_Group_Number(Default_WorkGroup))
			Serial.println("Save Inital group number success... <Request_Access_Network>");
		if (SN.Save_Area_Number(Init_Area))
			Serial.println("Save Inital area number success... <Request_Access_Network>");

		Serial.println("");
		Serial.println("Not registered to server, please send \"S\"	<Request_Access_Network>");
		// LED_NO_REGISTER;
	}
	while (SN.Verify_SN_Access_Network_Flag() == false)
	{
		iwdg_feed();
		while (Serial.available() > 0)
		{
			comdata += char(Serial.read());  //每次读一个char字符，并相加
			delay(2);
		}

		if (comdata.length() > 0)
		{
			comdata.toUpperCase();
			Serial.println(comdata);
			if (comdata == String("S"))
			{
				comdata = "";
				Serial.println("Start sending registration data to server <Request_Access_Network>");
				Message_Receipt.Report_General_Parameter();
			}

			iwdg_feed();
		}
		comdata = "";
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
	}
	gAccessNetworkFlag = true;
}

/*
 @brief   : 工程模式。用于在单机工作的情况下。手动卷膜，可以测量脉冲数、电机电流、上下限位等。
			通过按键1，可以测试重置行程。
 @para    : 无
 @return  : 无
 */
void Project_Debug(void)
{
	// while (1)
	// {
	// 	iwdg_feed();
	// 	for (size_t i = 0; i < 12; i++)
	// 	{

	// 	}

	// }

}

/*
 @brief   : 按键重置LORA参数
 @para    : 无
 @return  : 无
 */
void Key_Reset_LoRa_Parameter(void)
{
	// if (digitalRead(SW_FUN1) == LOW)
	// {
	// 	MyDelayMs(100);
	// 	if (digitalRead(SW_FUN1) == LOW)
	// 	{
	// 		MyDelayMs(3000);
	// 		iwdg_feed();
	// 		if (digitalRead(SW_FUN2) == LOW)
	// 		{
	// 			MyDelayMs(100);
	// 			if (digitalRead(SW_FUN2) == LOW)
	// 			{
	// 				LoRa_Para_Config.Clear_LoRa_Config_Flag();
	// 				Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
	// 				Serial.println("清除LoRa配置标志成功...<Key_Reset_LoRa_Parameter>");
	// 				iwdg_feed();
	// 			}
	// 		}
	// 	}
	// }
}

/*
 @brief   : 定时状态上报
 @para    : 无
 @return  : 无
 */
void Regular_status_report(void)
{
	if (gStateReportFlag)
	{
		Serial.println("开始定时状态上报");
		gStateReportFlag = false;
		Stop_Status_Report_Timing();

		/*得到随机值*/
		unsigned char random_1 = random(0, 255);
		unsigned char random_2 = random(0, 255);
		Serial.println(String("random_1 = ") + String(random_1, HEX));
		Serial.println(String("random_2 = ") + String(random_2, HEX));


		/*这里上报实时状态*/
		Message_Receipt.Working_Parameter_Receipt(false, 1, random_1, random_2);

		Start_Status_Report_Timing();
		Serial.println("Scheduled status reporting completed... <Regular_status_report>");
	}
}

/*
 @brief   : 状态改变上报
 @para    : 无
 @return  : 无
 */
void Change_status_report(void)
{
	if (DOStatus_Change)
	{
		Serial.println("开始DO状态改变上报 <Change_status_report>");
		DOStatus_Change = false;
		Get_receipt = true;

		/*得到随机值*/
		unsigned char random_1 = random(0, 255);
		unsigned char random_2 = random(0, 255);
		Serial.println(String("random_1 = ") + String(random_1, HEX));
		Serial.println(String("random_2 = ") + String(random_2, HEX));

		/*这里上报实时状态*/
		Message_Receipt.Working_Parameter_Receipt(false, 2, random_1, random_2);

		Serial.println("Scheduled status reporting completed... <Change_status_report>");
	}

	if (One_Cycle_Complete)
	{
		Serial.println("开始循环完成上报 <Change_status_report>");
		One_Cycle_Complete = false;
		Get_receipt = false;

		/*得到随机值*/
		unsigned char random_1 = random(0, 255);
		unsigned char random_2 = random(0, 255);

		/*这里上报完成一个轮次循环*/
		Message_Receipt.Irrigation_loop_Receipt(false, 1, random_1, random_2);

		Serial.println("Cycle status report completed... <Change_status_report>");
	}
}



/*
 @brief   : 灌溉分时开启V2
 @para    : 无
 @return  : 无
 */
void Irrigation_time_sharing_onV2()
{
//	iwdg_feed();
//	if (retryCnt > 0)//代表进入了循环灌溉
//	{
//		//Set_Irrigation_relay(11, ON);//开启循环时，Y12亮
//
//		if (Cyclic_intervalFlag)//循环时间间隔的标志位
//		{
//#if USE_RTC
//			Cyclic_intervalFlag = false;
//			gRTCTime_arrive_Flag = false;
//			Private_RTC.Set_Alarm();//设置RTC闹钟
//			Serial.println("开始循环间隔计时");
//			Cyclic_timing = true;
//#elif USE_TIMER
//			Cyclic_intervalFlag = false;
//			gTime_arrive_Flag = false;
//			//开始循环间隔计时
//			Irrigation_time = Cyclic_interval;
//			Start_Timer4();
//			Serial.println("开始循环间隔计时");
//			Cyclic_timing = true;
//#else
//#endif // USE_RTC
//		}
//
//		if (DO_intervalFlag)
//		{
//			DO_intervalFlag = false;
//			gTime_arrive_Flag = false;
//			//开始循环间隔计时
//			Irrigation_time = Worktime_backups[0];
//			Start_Timer4();
//			Serial.println("开始单路间隔时间计时");
//			DO_interval_timing = true;
//		}
//
//		if (Cyclic_timing)//正在进行循环时间间隔的标志位
//		{
//#if USE_RTC
//			if (gRTCTime_arrive_Flag)
//			{
//				Serial.println("循环时间间隔结束");
//				Serial.println(">>>>>>>>>>>>");
//				Cyclic_timing = false;
//				gRTCTime_arrive_Flag = false;
//			}
//#elif USE_TIMER
//			if (gTime_arrive_Flag)
//			{
//				Serial.println("循环时间间隔结束");
//				Serial.println(">>>>>>>>>>>>");
//				Stop_Timer4();
//				Cyclic_timing = false;
//				gTime_arrive_Flag = false;
//			}
//#else
//#endif // USE_RTC
//		}
//		else if (DO_interval_timing)
//		{
//			if (gTime_arrive_Flag)
//			{
//				Serial.println("间隔等待时间到达");
//				Stop_Timer4();
//				DO_interval_timing = false;
//				gTime_arrive_Flag = false;
//				Irrigation_use = false;
//			}
//		}
//		else
//		{
//			//for (size_t i = 0; i < fornum; i++)//一轮循环需要的循环次数，（例如开4路，每次开2路，fornum=2）
//			while(fornum)
//			{
//				/*unsigned char GET_DO_ON = 0;*/
//				for (size_t c = 0; c < 12; c++)
//				{
//					//iwdg_feed();
//					if (Worktime[(2 * c) + 1] > 0)//代表该路需要开启
//					{
//						GET_DO_ON++;
//						Set_Irrigation_relay(c, ON);//开启该路
//
//						//if (i == fornum - 1)//代表一轮循环的最后一个循环
//						if (fornum == 1)//代表一轮循环的最后一个循环
//						{
//							if (Last_full)//代表一轮循环的最后一个循环能开满
//							{
//								if (GET_DO_ON == DO_Num)//代表已经开启了足够的DO
//								{
//									if (Judge_ONorOFF(c, DO_Num, true))
//									{
//										return;
//									}
//									//Serial.println("1111111"); delay(1000); GET_DO_ON = 0;
//									//if (!Irrigation_use)
//									//{
//									//	Irrigation_use = true;
//									//	gTime_arrive_Flag = false;
//									//	Irrigation_time = Worktime[(2 * c) + 1];
//									//	Start_Timer4();
//									//	//Set_Irrigation_relay(i / 2, ON);
//									//	DOStatus_Change = true;//状态改变标志位已改变
//
//									//	Serial.println(String("开始第") + c + "路计时");
//									//}
//
//									//if (gTime_arrive_Flag)
//									//{
//									//	Serial.println(String("第") + c + "路时间到达");
//									//	Serial.println("------");
//									//	fornum--;
//									//	GET_DO_ON = 0;//
//									//	Complete_Num += DO_Num;//完成的个数+= DO_Num
//									//	Stop_Timer4();
//									//	for (size_t j = 0; j < c + 1; j++)
//									//	{
//									//		Worktime[2 * j] = 0;
//									//		Worktime[(2 * j) + 1] = 0;
//									//		Set_Irrigation_relay(j, OFF);
//									//	}
//									//	DOStatus_Change = true;//状态改变标志位已改变
//
//									//	gTime_arrive_Flag = false;
//									//	Irrigation_use = false;
//
//									//	//这里判断是否到达循环次数
//									//	Judge_Cycle_completion();
//									//	return;
//									//}
//									break;
//								}
//							}
//							else//代表一轮循环的最后一个循环不能开满
//							{
//								if (GET_DO_ON == Last_num)//代表已经开启了足够的DO
//								{
//									if (Judge_ONorOFF(c, Last_num, true))
//									{
//										return;
//									}
//									//Serial.println("2222222"); delay(1000); GET_DO_ON = 0;
//									//if (!Irrigation_use)
//									//{
//									//	Irrigation_use = true;
//									//	gTime_arrive_Flag = false;
//									//	Irrigation_time = Worktime[(2 * c) + 1];
//									//	Start_Timer4();
//									//	//Set_Irrigation_relay(i / 2, ON);
//									//	DOStatus_Change = true;//状态改变标志位已改变
//
//									//	Serial.println(String("开始第") + c + "路计时");
//									//}
//
//									//if (gTime_arrive_Flag)
//									//{
//									//	Serial.println(String("第") + c + "路时间到达");
//									//	Serial.println("------");
//									//	fornum--;
//									//	GET_DO_ON = 0;//
//									//	Complete_Num += Last_num;//完成的个数+= DO_Num
//									//	Stop_Timer4();
//									//	for (size_t j = 0; j < c + 1; j++)
//									//	{
//									//		Worktime[2 * j] = 0;
//									//		Worktime[(2 * j) + 1] = 0;
//									//		Set_Irrigation_relay(j, OFF);
//									//	}
//									//	DOStatus_Change = true;//状态改变标志位已改变
//
//									//	gTime_arrive_Flag = false;
//									//	Irrigation_use = false;
//
//									//	//这里判断是否到达循环次数
//									//	Judge_Cycle_completion();
//									//	return;
//									//}
//									break;
//								}
//							}
//						}
//						else
//						{
//							if (GET_DO_ON == DO_Num)//代表已经开启了足够的DO
//							{
//								if (Judge_ONorOFF(c, DO_Num, false))
//								{
//									return;
//								}
//								//Serial.println("3333333"); delay(1000); GET_DO_ON = 0;
//								//if (!Irrigation_use)
//								//{
//								//	Irrigation_use = true;
//								//	gTime_arrive_Flag = false;
//								//	Irrigation_time = Worktime[(2 * c) + 1];
//								//	Start_Timer4();
//								//	//Set_Irrigation_relay(i / 2, ON);
//								//	DOStatus_Change = true;//状态改变标志位已改变
//
//								//	Serial.println(String("开始第") + c + "路计时");
//								//}
//
//								//if (gTime_arrive_Flag)
//								//{
//								//	Serial.println(String("第") + c + "路时间到达");
//								//	Serial.println("------");
//								//	fornum--;
//								//	GET_DO_ON = 0;//
//								//	Complete_Num+= DO_Num;//完成的个数+= DO_Num
//								//	Stop_Timer4();
//								//	for (size_t j = 0; j < c + 1; j++)
//								//	{
//								//		Worktime[2 * j] = 0;
//								//		Worktime[(2 * j) + 1] = 0;
//								//		Set_Irrigation_relay(j, OFF);
//								//	}
//								//	DOStatus_Change = true;//状态改变标志位已改变
//
//								//	gTime_arrive_Flag = false;
//								//	Irrigation_use = false;
//								//	return;
//								//}
//								break;
//							}
//						}
//					}
//				}
//				return;
//			}
//		}
//	}
//	else
//	{
//		//Set_Irrigation_relay(11, OFF);//退出循环时，Y12灭
//	}
}

/*
 @brief   : 按键启动循环灌溉v2
 @para    : 无
 @return  : 无
 */
void Key_cycle_irrigationV2(void)
{
//	if (KeyDI7_num % 2)//代表需要启动
//	{
//		Get_receipt = true;
//	}
//	else
//	{
//		//Get_receipt = false;
//	}
//#if USE_COM
//	while (Serial.available() > 0)
//	{
//		comdata += char(Serial.read());  //每次读一个char字符，并相加
//		delay(2);
//	}
//
//	if (comdata.length() > 0)
//	{
//		comdata.toUpperCase();
//		Serial.println(comdata);
//		if (comdata == String("B"))
//		{
//			comdata = "";
//			KeyDI7_num++;
//			Serial.println("Start circulating irrigation button press <Key_cycle_irrigationV2>");
//			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
//			Serial.println("");
//			if (KeyDI7_num % 2)//代表需要启动
//			{
//				Stop_Timer4();//先停止计时
//				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
//				Cyclic_intervalFlag = false;//
//				gRTCTime_arrive_Flag = false;//
//				gTime_arrive_Flag = false; //
//				Cyclic_timing = false;//
//				Irrigation_use = false;//
//				Need_Num = 0;	//需要开启继电器的个数
//				Complete_Num = 0;//完成开启的个数
//
//				for (size_t i = 0; i < 8; i++)
//				{
//					Worktime[2 * i] = 0;
//					Worktime[(2 * i) + 1] = 0;
//					Worktime_backups[2 * i] = 0;
//					Worktime_backups[(2 * i) + 1] = 0;
//					Set_Irrigation_relay(i, OFF);
//				}
//
//				OpenSec = 2;//开启时间为20s
//				DO_Interval = 2;//间隔时间为2s
//				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）
//				DO_Num = 1;//每次开启1路
//				retryCnt = 0x3;//65535次循环
//
//				for (size_t i = 0; i < 8; i++)
//				{
//					Worktime[2 * i] = DO_Interval;
//					Worktime[(2 * i) + 1] = OpenSec;
//					Worktime_backups[2 * i] = DO_Interval;
//					Worktime_backups[(2 * i) + 1] = OpenSec;
//				}
//				Need_Num = 8;
//			}
//			else//代表关闭
//			{
//				Stop_Timer4();//先停止计时
//				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
//				Cyclic_intervalFlag = false;//
//				gRTCTime_arrive_Flag = false;//
//				gTime_arrive_Flag = false; //
//				Cyclic_timing = false;//
//				Irrigation_use = false;//
//				Need_Num = 0;	//需要开启继电器的个数
//				Complete_Num = 0;//完成开启的个数
//
//				for (size_t i = 0; i < 8; i++)
//				{
//					Worktime[2 * i] = 0;
//					Worktime[(2 * i) + 1] = 0;
//					Worktime_backups[2 * i] = 0;
//					Worktime_backups[(2 * i) + 1] = 0;
//					Set_Irrigation_relay(i, OFF);
//				}
//
//				OpenSec = 0;//开启时间为0s
//				DO_Interval = 0;//间隔时间为0s
//				Cyclic_interval = 0;//循环间隔为0s（0）
//				DO_Num = 1;//每次开启1路
//				retryCnt = 0;//0次循环
//			}
//		}
//		else
//		{
//			//comdata = "";
//		}
//		iwdg_feed();
//	}
//#elif USE_KEY
//	if (digitalRead(DI7) == HIGH)
//	{
//		iwdg_feed();
//		delay(100);
//		if (digitalRead(DI7) == HIGH)
//		{
//			KeyDI7_num++;
//			Serial.println("Start circulating irrigation button press <Key_cycle_irrigationV2>");
//			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
//			Serial.println("");
//			/*等待按键释放*/
//			while (digitalRead(DI7) == HIGH)
//			{
//
//			}
//
//			if (KeyDI7_num % 2)//代表需要启动
//			{
//				Stop_Timer4();//先停止计时
//				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
//				Cyclic_intervalFlag = false;//
//				gRTCTime_arrive_Flag = false;//
//				gTime_arrive_Flag = false; //
//				Cyclic_timing = false;//
//				Irrigation_use = false;//
//				Need_Num = 0;	//需要开启继电器的个数
//				Complete_Num = 0;//完成开启的个数
//
//				for (size_t i = 0; i < 8; i++)
//				{
//					Worktime[2 * i] = 0;
//					Worktime[(2 * i) + 1] = 0;
//					Worktime_backups[2 * i] = 0;
//					Worktime_backups[(2 * i) + 1] = 0;
//					Set_Irrigation_relay(i, OFF);
//				}
//
//				OpenSec = 20;//开启时间为20s
//				DO_Interval = 2;//间隔时间为2s
//				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）
//				DO_Num = 1;//每次开启1路
//				retryCnt = 0xFFFF;//65535次循环
//
//				for (size_t i = 0; i < 8; i++)
//				{
//					Worktime[2 * i] = DO_Interval;
//					Worktime[(2 * i) + 1] = OpenSec;
//					Worktime_backups[2 * i] = DO_Interval;
//					Worktime_backups[(2 * i) + 1] = OpenSec;
//				}
//				Need_Num = 8;
//			}
//			else//代表关闭
//			{
//				Stop_Timer4();//先停止计时
//				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
//				Cyclic_intervalFlag = false;//
//				gRTCTime_arrive_Flag = false;//
//				gTime_arrive_Flag = false; //
//				Cyclic_timing = false;//
//				Irrigation_use = false;//
//				Need_Num = 0;	//需要开启继电器的个数
//				Complete_Num = 0;//完成开启的个数
//
//				for (size_t i = 0; i < 8; i++)
//				{
//					Worktime[2 * i] = 0;
//					Worktime[(2 * i) + 1] = 0;
//					Worktime_backups[2 * i] = 0;
//					Worktime_backups[(2 * i) + 1] = 0;
//					Set_Irrigation_relay(i, OFF);
//				}
//
//				OpenSec = 0;//开启时间为0s
//				DO_Interval = 0;//间隔时间为0s
//				Cyclic_interval = 0;//循环间隔为0s（0）
//				DO_Num = 1;//每次开启1路
//				retryCnt = 0;//0次循环
//			}
//		}
//	}
//#else
//
//#endif
}

/*
 @brief   : 串口设置循环间隔
 @para    : 无
 @return  : 无
 */
void Com_Set_Cyclic_interval(void)
{
	while (Serial.available() > 0)
	{
		comdata += char(Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Serial.println(comdata);
		if (comdata.startsWith("SET:"))
		{
			comdata.remove(0, 4);
			//Serial.println(comdata);
			Cyclic_interval = comdata.toInt();
			//Serial.println(String("Cyclic_interval = ") + Cyclic_interval);
			unsigned char Cyclic_interval_1 = (Cyclic_interval >> 8) & 0XFF;
			unsigned char Cyclic_interval_2 = Cyclic_interval & 0XFF;
			if (InitState.Save_CyclicInterval(Cyclic_interval_1, Cyclic_interval_2))
			{
				Serial.println("设置并保存参数成功 <Com_Set_Cyclic_interval>");
			}
			else
			{
				Serial.println("设置并保存参数失败!!! <Com_Set_Cyclic_interval>");
			}
			Cyclic_interval = InitState.Read_CyclicInterval();
			Serial.println(String("Cyclic_interval = ") + Cyclic_interval);
			comdata = "";
		}
		else
		{
			comdata = "";
			Serial.println("输入错误 <Com_Set_Cyclic_interval>");
		}
	}
}

/*
 @brief   : 判断灌溉循环是否完成
 @para    : 无
 @return  : 无
 */
void Judge_Cycle_completion()
{
	/*if (Complete_Num == Need_Num)
	{
		Serial.println("已完成1次灌溉循环");
		Serial.println();
		Complete_Num = 0;
		retryCnt--;
		for (size_t i = 0; i < 24; i++)
		{
			Worktime[i] = Worktime_backups[i];
		}
		fornum = fornum_backups;


		if (retryCnt > 0)
		{
			Cyclic_intervalFlag = true;
			DO_intervalFlag = false;
			Serial.println("继续循环...");
			Serial.println(String("剩余循环次数为 = ") + retryCnt);
		}
		else
		{
			Cyclic_intervalFlag = false;
			Serial.println("完成所有的灌溉循环");
			Serial.println("(｡◕ˇ∀ˇ◕）");
		}
	}*/
}

/*
 @brief   : 判断开启或关闭
 @para    : 1、当前for循环的c
			2、完成的个数C_N（Complete_Num）
			3、是否需要进行判断循环完成的标志位
 @return  : 需要return则返回true，不需要return返回false
 */
// bool Judge_ONorOFF(unsigned char c,unsigned char C_N,bool Need_Judge)
// {
// 	GET_DO_ON = 0;

// 	if (!Irrigation_use)
// 	{
// 		Irrigation_use = true;
// 		gTime_arrive_Flag = false;
// 		Irrigation_time = Worktime[(2 * c) + 1];
// 		Start_Timer4();
// 		//Set_Irrigation_relay(i / 2, ON);
// 		DOStatus_Change = true;//状态改变标志位已改变

// 		Serial.println(String("开始第") + c + "路计时");
// 	}

// 	if (gTime_arrive_Flag)
// 	{
// 		Serial.println(String("第") + c + "路时间到达");
// 		Serial.println("------");
// 		fornum--;
// 		GET_DO_ON = 0;//
// 		Complete_Num += C_N;//完成的个数+= C_N
// 		Stop_Timer4();
// 		for (size_t j = 0; j < c + 1; j++)
// 		{
// 			Worktime[2 * j] = 0;
// 			Worktime[(2 * j) + 1] = 0;
// 			Set_Irrigation_relay(j, OFF);
// 		}
// 		DOStatus_Change = true;//状态改变标志位已改变

// 		gTime_arrive_Flag = false;
// 		Irrigation_use = false;
// 		DO_intervalFlag = true;

// 		if (Need_Judge)
// 		{
// 			Judge_Cycle_completion();
// 		}

// 		return true;
// 	}
// 	return false;
// }



/*
 @brief   : 灌溉分时开启V3
 @para    : 无
 @return  : 无
 */
void Irrigation_time_sharing_onV3()
{
	iwdg_feed();
	if (Complete_Num!=2)//等待本轮循环结束
	{
		for (unsigned char i = 0; i < 2; i++)
		{
			iwdg_feed();
			if (!DO_Set[i])//该路DO未被设置
			{
				//Serial.println(String("第") + i + "路未被设置");
				DO_Set[i] = true;//DO状态已被设置

				if (Worktime[i] > 0 && retryCnt[i] > 0)
				{
					Set_Irrigation_relay(i, ON);
					DO_WayOpen[i] = true;//该路DO已打开
					DOStatus_Change = true;//状态改变标志位已改变
					DO_WayOpentime[i] = gIrrigationNum;//记录该路DO的打开时间
					DO_WayClosetime[i] = DO_WayOpentime[i] + Worktime[i];//计算出该路DO的关闭时间
					Serial.println(String("第") + i + "路已设置并开启");
					Serial.println(String("DO_WayClosetime[") + i + "] = "+ DO_WayClosetime[i]);
					Serial.flush();
					
					if (WorkInterval[i] != 0)//该路有间隔时间
					{
						return;
					}
				}
				else
				{
					//表示不用开启该路
					Serial.println(String("第") + i + "路无需开启");
					DO_WayComplete[i] = true;//该路已完成
					Complete_Num++;//完成了一路，不用开启代表直接完成
					Serial.println(String("完成个数 = ") + Complete_Num);
					Serial.flush();
					if (Complete_Num == 2)
					{
						Cyclic_intervalFlag = true;//需要循环计时
						Serial.println("已完成本轮循环");
						One_Cycle_Complete = true;//一轮循环已完成
					}
				}
			}
			else
			{
				switch (i)
				{
				case 0:if (DO_OFF(i)) {return;}	break;
				case 1:if (DO_OFF(i)) {return;}	break;
				// case 2:if (DO_OFF(i)) {return;}	break;
				// case 3:if (DO_OFF(i)) {return;}	break;
				// case 4:if (DO_OFF(i)) {return;}	break;
				// case 5:if (DO_OFF(i)) {return;}	break;
				// case 6:if (DO_OFF(i)) {return;}	break;
				// case 7:if (DO_OFF(i)) {return;}	break;
				// case 8:if (DO_OFF(i)) {return;}	break;
				// case 9:if (DO_OFF(i)) {return;}	break;
				// case 10:if (DO_OFF(i)) {return;}break;
				// case 11:if (DO_OFF(i)) {return;}break;
				default:Serial.println("default");	break;
				}
			}
		}
	}
	else//完成了一轮循环
	{
		Stop_Timer4();

		if (Cyclic_intervalFlag)
		{
			Cyclic_intervalFlag = false;
			gRTCTime_arrive_Flag = false;
			Private_RTC.Set_Alarm();//设置RTC闹钟
			Serial.println("开始循环间隔计时");
			Cyclic_timing = true;
		}

		if (Cyclic_timing)//正在进行循环时间间隔的标志位
		{
			if (gRTCTime_arrive_Flag)
			{
				Serial.println("循环时间间隔结束");
				Serial.println(">>>>>>>>>>>>");
				Cyclic_timing = false;
				gRTCTime_arrive_Flag = false;
				for (size_t i = 0; i < 2; i++)
				{
					DO_Set[i] = false;//该路DO未设置
					DO_WayComplete[i] = false;//该路DO未完成
					DO_WayOpen[i] = false;//该路DO已关闭
					if (retryCnt[i] > 0)//代表还有循环
					{
						Complete_Num = 0;
					}
				}

				if (Complete_Num == 0)
				{
					Serial.println("下一轮循环开始");
					Serial.println(">>>>>>>>>>>>");
					Start_Timer4();
				}
				else
				{
					Serial.println("完成所有的灌溉循环");
					Serial.println("(｡◕ˇ∀ˇ◕）");
				}
			}
		}
	}
}

bool DO_OFF(unsigned char i)
{
	if (!DO_WayComplete[i])//该路没完成
	{
		if (DO_WayOpen[i])//该路DO已打开
		{
			if (gIrrigationNum >= DO_WayClosetime[i])
			{
				Set_Irrigation_relay(i, OFF);
				DO_WayOpen[i] = false;//该路DO已关闭
				DOStatus_Change = true;//状态改变标志位已改变
				Serial.println(String("第") + i + "路时间已到达并关闭");
				Serial.flush();
				if (WorkInterval[i] != 0)//间隔时间不为0
				{
					DO_WayInterval[i] = true;//需要进行间隔
					//DO_WayComplete[i] = true;//该路已完成
					DO_WayIntervalBengin[i] = gIrrigationNum;//记录DO的间隔开始时间
					DO_WayIntervalEnd[i] = DO_WayIntervalBengin[i] + WorkInterval[i];//记录DO的间隔结束时间
					Serial.println(String("DO_WayIntervalEnd[") + i + "] = " + DO_WayIntervalEnd[i]);
					Serial.flush();
				}
				else//间隔时间为0
				{
					retryCnt[i]--;//循环的次数-1
					Complete_Num++;//完成了一路
					Serial.println(String("完成个数 = ") + Complete_Num);
					Serial.flush();
					if (Complete_Num == 2)
					{
						Serial.println("已完成本轮循环");
						Cyclic_intervalFlag = true;//需要循环计时
						One_Cycle_Complete = true;//一轮循环已完成
					}
					DO_WayInterval[i] = false;//不需要进行间隔
					DO_WayComplete[i] = true;//16路DO完成的标志位
					return false;
				}
			}
		}
		if (DO_WayInterval[i])//该路DO需要间隔
		{
			if (gIrrigationNum >= DO_WayIntervalEnd[i])
			{
				DO_WayComplete[i] = true;//该路已完成
				retryCnt[i]--;//循环的次数-1
				Complete_Num++;//完成了一路
				Serial.println(String("完成个数 = ") + Complete_Num);
				Serial.flush();
				if (Complete_Num == 2)
				{
					Serial.println("已完成本轮循环");
					Cyclic_intervalFlag = true;//需要循环计时
					One_Cycle_Complete = true;//一轮循环已完成
				}
				DO_WayComplete[i] = true;//该路已完成
				Serial.println(String("第") + i + "路间隔时间到达");
				Serial.flush();
				return false;
			}
			return true;
		}
		if (WorkInterval[i] != 0)
		{
			return true;
		}
	}
	return false;
}


/*
 @brief   : 按键启动循环灌溉v3
 @para    : 无
 @return  : 无
 */
void Key_cycle_irrigationV3(void)
{
	if (KeyDI7_num % 2)//代表需要启动
	{
		Get_receipt = true;
	}
	else
	{
		//Get_receipt = false;
	}
#if USE_COM
	while (Serial.available() > 0)
	{
		comdata += char(Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Serial.println(comdata);
		if (comdata == String("B"))
		{
			comdata = "";
			KeyDI7_num++;
			Serial.println("Start circulating irrigation button press <Key_cycle_irrigationV2>");
			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
			Serial.println("");
			Stop_Timer4();//先停止计时
			rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？

			Cyclic_intervalFlag = false;//循环时间间隔的标志位
			gRTCTime_arrive_Flag = false;//RTC时间到达的标志位
			gTime_arrive_Flag = false; //定时器时间到达的标志位
			Cyclic_timing = false;//正在进行循环时间间隔的标志位
			Irrigation_use = false;//正在灌溉的标志位
			//OpenSec = 0;//开启的时间
			//DO_Interval = 0;//单个的间隔时间
			//DO_Num = 0;//一次性开启的DO数量
			//retryCnt = 0;//循环次数（）
			Cyclic_interval = 0;//循环间隔时间
			//fornum = 0;//一轮循环需要的循环次数，（例如开4路，每次开2路，fornum=2）
			//fornum_backups = 0;//一轮循环需要的循环次数的备份，（例如开4路，每次开2路，fornum=2）
			//Last_full = false;//最后一轮循环是否能开满，（例如开5路，每次开2路，最后一轮开不满）
			//Last_num = 0;//最后一轮循环开不满时需要开启的个数
			DO_intervalFlag = false;//单个间隔时间的标志位
			DO_interval_timing = false;//正在进行单个间隔时间的标志位
			DOStatus_Change = false;//DO的状态改变标志位
			One_Cycle_Complete = false;//一轮循环完成的标志位

			Need_Num = 0;	//需要开启继电器的个数
			Complete_Num = 0;//完成开启的个数

			GET_DO_ON = 0;//开启的数量

			for (size_t i = 0; i < 2; i++)
			{
				Worktime[i] = 0;//16个开启时间
				WorkInterval[i] = 0;//16个间隔时间
				Worktime_backups[i] = 0;//16个开启时间的备份
				WorkInterval_backups[i] = 0;//16个间隔时间的备份
				retryCnt[i] = 0;//循环次数（）
				DO_WayOpentime[i] = 0;//16路DO的开始时间
				DO_WayClosetime[i] = 0;//16路DO的关闭时间
				DO_WayIntervalBengin[i] = 0;//16路DO的间隔开始时间
				DO_WayIntervalEnd[i] = 0;//16路DO的间隔结束时间
				DO_WayOpen[i] = false; //16路DO打开的标志位
				DO_WayInterval[i] = false;//16路DO间隔标志位
				DO_WayComplete[i] = false;//16路DO完成的标志位
				DO_Set[i] = false;//DO设置的标志位
				Set_Irrigation_relay(i, OFF);
			}
			if (KeyDI7_num % 2)//代表需要启动
			{
				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）

				for (size_t i = 0; i < 2; i++)
				{
					Worktime[i] = 20;//开启时间为20s
					WorkInterval[i] = 60;//间隔时间为60s
					retryCnt[i] = 0xFFFF;//65535次循环
				}
				Start_Timer4();
			}
			else//代表关闭
			{
				Cyclic_interval = 0;//循环间隔为0小时
				for (size_t i = 0; i < 2; i++)
				{
					Worktime[i] = 0;//开启时间为0s
					WorkInterval[i] = 0;//间隔时间为0s
					retryCnt[i] = 0x00;//0次循环
				}
			}
		}
		else
		{
			//comdata = "";
		}
		iwdg_feed();
	}
#elif USE_KEY
	if (digitalRead(DI7) == HIGH)
	{
		iwdg_feed();
		delay(100);
		if (digitalRead(DI7) == HIGH)
		{
			KeyDI7_num++;
			Serial.println("Start circulating irrigation button press <Key_cycle_irrigationV2>");
			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
			Serial.println("");
			Stop_Timer4();//先停止计时
			rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？

			Cyclic_intervalFlag = false;//循环时间间隔的标志位
			gRTCTime_arrive_Flag = false;//RTC时间到达的标志位
			gTime_arrive_Flag = false; //定时器时间到达的标志位
			Cyclic_timing = false;//正在进行循环时间间隔的标志位
			Irrigation_use = false;//正在灌溉的标志位
			//OpenSec = 0;//开启的时间
			//DO_Interval = 0;//单个的间隔时间
			//DO_Num = 0;//一次性开启的DO数量
			//retryCnt = 0;//循环次数（）
			Cyclic_interval = 0;//循环间隔时间
			//fornum = 0;//一轮循环需要的循环次数，（例如开4路，每次开2路，fornum=2）
			//fornum_backups = 0;//一轮循环需要的循环次数的备份，（例如开4路，每次开2路，fornum=2）
			//Last_full = false;//最后一轮循环是否能开满，（例如开5路，每次开2路，最后一轮开不满）
			//Last_num = 0;//最后一轮循环开不满时需要开启的个数
			DO_intervalFlag = false;//单个间隔时间的标志位
			DO_interval_timing = false;//正在进行单个间隔时间的标志位
			DOStatus_Change = false;//DO的状态改变标志位
			One_Cycle_Complete = false;//一轮循环完成的标志位

			Need_Num = 0;	//需要开启继电器的个数
			Complete_Num = 0;//完成开启的个数

			GET_DO_ON = 0;//开启的数量

			for (size_t i = 0; i < 2; i++)
			{
				Worktime[i] = 0;//16个开启时间
				WorkInterval[i] = 0;//16个间隔时间
				Worktime_backups[i] = 0;//16个开启时间的备份
				WorkInterval_backups[i] = 0;//16个间隔时间的备份
				retryCnt[i] = 0;//循环次数（）
				DO_WayOpentime[i] = 0;//16路DO的开始时间
				DO_WayClosetime[i] = 0;//16路DO的关闭时间
				DO_WayIntervalBengin[i] = 0;//16路DO的间隔开始时间
				DO_WayIntervalEnd[i] = 0;//16路DO的间隔结束时间
				DO_WayOpen[16] = false; //16路DO打开的标志位
				DO_WayInterval[i] = false;//16路DO间隔标志位
				DO_WayComplete[i] = false;//16路DO完成的标志位
				DO_Set[i] = false;//DO设置的标志位
				Set_Irrigation_relay(i, OFF);
			}

			/*等待按键释放*/
			while (digitalRead(DI7) == HIGH)
			{

			}

			if (KeyDI7_num % 2)//代表需要启动
			{
				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）

				for (size_t i = 0; i < 2; i++)
				{
					Worktime[i] = 20;//开启时间为20s
					WorkInterval[i] = 60;//间隔时间为60s
					retryCnt[i] = 0xFFFF;//65535次循环
				}
				Start_Timer4();
			}
			else//代表关闭
			{
				Cyclic_interval = 0;//循环间隔为0小时
				for (size_t i = 0; i < 2; i++)
				{
					Worktime[i] = 0;//开启时间为0s
					WorkInterval[i] = 0;//间隔时间为0s
					retryCnt[i] = 0x00;//0次循环
				}
			}
		}
	}
#else

#endif
}
