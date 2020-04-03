/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/8/27
 *
 * 该文件的作用是接收服务器通过LoRa无线模块发送过来的指令数据，然后解析这些指令。指令包括通用指令
 * （绑定SN，设置区域号，设置工作组号，查询本机状态等），私有指令（卷膜机重置行程，设置开度卷膜
 * 设置卷膜工作阈值等）。接收的指令都要校验CRC8，有些指令要校验区域号或工作组号本机才会执行相应
 * 功能。
 * 头文件中提供了各个类的公共接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Command_Analysis.h"
#include <libmaple/iwdg.h>
#include "LoRa.h"
#include "User_CRC8.h"
#include "Memory.h"
#include "receipt.h"
#include "fun_periph.h"
#include "Set_coil.h"
#include "Private_Timer.h"
#include "Private_RTC.h"
//#include "ModbusSerial.h"

Command_Analysis LoRa_Command_Analysis;

unsigned char gReceiveCmd[128];   //接收LoRa数据缓存
unsigned char gReceiveLength;     //接收LoRa数据长度
bool gAccessNetworkFlag = true;   //是否已经注册到服务器标志位
// bool gStopWorkFlag = false;       //是否要强制停止标志位
bool gMassCommandFlag = false;    //接收的消息是否是群发标志位

bool gIsHandleMsgFlag = true;     //是否接收到LoRa消息然后解析处理，还是只接收不解析处理（刚上电LoRa模块发的厂家信息）
bool Get_receipt = true;	//是否接收到回执之后的回执，接收到为true，未接收到为false
unsigned char randomId_1;	//
unsigned char randomId_2;

unsigned int Worktime[16];//16个开启时间
unsigned int Worktime_backups[16];//16个开启时间的备份
unsigned int WorkInterval[16];//16个间隔时间
unsigned int WorkInterval_backups[16];//16个间隔时间的备份
unsigned int retryCnt[16];//循环次数（）
unsigned int DO_WayOpentime[16];//16路DO的开始时间
unsigned int DO_WayClosetime[16];//16路DO的关闭时间
unsigned int DO_WayIntervalBengin[16];//16路DO的间隔开始时间
unsigned int DO_WayIntervalEnd[16];//16路DO的间隔结束时间
bool DO_WayOpen[16];//16路DO打开的标志位
bool DO_WayInterval[16];//16路DO间隔标志位
bool DO_WayComplete[16];//16路DO完成的标志位
bool Irrigation_use = false;//正在灌溉的标志位
bool DO_Set[16] = {true};//DO设置的标志位
//unsigned int OpenSec = 0;//开启的时间
//unsigned int DO_Interval = 0;//单个的间隔时间
//unsigned int DO_Num = 0;//一次性开启的DO数量
unsigned char Need_Num = 0;//一轮循环需要开启的DO数量
unsigned char Complete_Num = 0;//一轮循环完成开启的DO数量
unsigned int Cyclic_interval = 0;//循环间隔时间
//unsigned char fornum = 0;//一轮循环需要的循环次数，（例如开4路，每次开2路，fornum=2）
//unsigned char fornum_backups = 0;//一轮循环需要的循环次数的备份，（例如开4路，每次开2路，fornum=2）
//bool Last_full = false;//最后一轮循环是否能开满，（例如开5路，每次开2路，最后一轮开不满）
//unsigned char Last_num = 0;//最后一轮循环开不满时需要开启的个数

extern bool DOStatus_Change;//DO的状态改变标志位
extern bool One_Cycle_Complete;//一轮循环完成的标志位
extern bool Cyclic_intervalFlag;//循环时间间隔的标志位
extern bool Cyclic_timing;//正在进行循环时间间隔的标志位
extern bool DO_intervalFlag;//单个间隔时间的标志位
extern bool DO_interval_timing;//正在进行单个间隔时间的标志位
extern bool gRTCTime_arrive_Flag;//RTC时间到达的标志位
extern unsigned int KeyDI7_num;//按键DI7按下的次数
extern unsigned char GET_DO_ON;//开启的数量


/*
 @brief   : 从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
			每条指令以0xFE为帧头，0x0D 0x0A 0x0D 0x0A 0x0D 0x0A，6个字节为帧尾。最大接受指令长度为128字节，超过将清空接收的数据和长度。
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Receive_LoRa_Cmd(void)
{
	unsigned char EndNum = 0;  //帧尾数量计数值
	bool EndNumFlag = false;  //检测到第一个帧尾标志位
	bool ReceiveEndFlag = false;  //正确接收到一笔数据标志位
	unsigned char FrameHeadDex = 0;
	gReceiveLength = 0;
	iwdg_feed();

	while (LoRa_Serial.available() > 0)
	{
		iwdg_feed();
		gReceiveCmd[gReceiveLength++] = LoRa_Serial.read();
		delay(3);
		Serial.print(gReceiveCmd[gReceiveLength - 1], HEX);
		Serial.print(" ");

		/*数据超出可以接收的范围*/
		if (gReceiveLength >= 128)
		{
			iwdg_feed();
			gReceiveLength = 0;
			Serial.println("数据超出可以接收的范围");
			delay(1000);
			memset(gReceiveCmd, 0x00, sizeof(gReceiveCmd));
		}

		//记录帧头所在接收的数据包里的位置（因为噪音的干扰，第一个字节可能不是帧头）
		if (gReceiveCmd[gReceiveLength - 1] == 0xFE)
		{
			FrameHeadDex = gReceiveLength - 1;
		}

		/*验证帧尾: 0D 0A 0D 0A 0D 0A*/
		if (EndNumFlag == false)
		{
			if (gReceiveCmd[gReceiveLength - 1] == 0x0D) {  //如果检测到第一个帧尾
				EndNumFlag = true;
				//Serial.println("检测到第一个帧尾");
			}
		}
		/*接收校验剩余的帧尾*/
		if (EndNumFlag == true)
		{
			switch (EndNum)
			{
			case 0: gReceiveCmd[gReceiveLength - 1] == 0x0D ? EndNum += 1 : EndNum = 0; if (EndNum == 0) EndNumFlag = false; break;
			case 1: gReceiveCmd[gReceiveLength - 1] == 0x0A ? EndNum += 1 : EndNum = 0; if (EndNum == 0) EndNumFlag = false; break;
			case 2: gReceiveCmd[gReceiveLength - 1] == 0x0D ? EndNum += 1 : EndNum = 0; if (EndNum == 0) EndNumFlag = false; break;
			case 3: gReceiveCmd[gReceiveLength - 1] == 0x0A ? EndNum += 1 : EndNum = 0; if (EndNum == 0) EndNumFlag = false; break;
			case 4: gReceiveCmd[gReceiveLength - 1] == 0x0D ? EndNum += 1 : EndNum = 0; if (EndNum == 0) EndNumFlag = false; break;
			case 5: gReceiveCmd[gReceiveLength - 1] == 0x0A ? EndNum += 1 : EndNum = 0; if (EndNum == 0) EndNumFlag = false; break;
			}
		}
		if (EndNum == 6)  //帧尾校验正确
		{
			EndNum = 0;
			EndNumFlag = false;
			ReceiveEndFlag = true;
			Serial.println("Get frame end... <Receive_LoRa_Cmd>");
			Serial.flush();
			break;
		}

		/*验证帧尾: 0D 0A 0D 0A 0D 0A*/ //另一种验证帧尾的方式
		//if (gReceiveCmd[gReceiveLength - 1] == 0x0D && (EndNum % 2 == 0))  //如果检测到第一个帧尾
		//	EndNum += 1;
		//else if (gReceiveCmd[gReceiveLength - 1] == 0x0A && (EndNum % 2 == 1))
		//	EndNum += 1;
		//else
		//	EndNum = 0;

		//if (EndNum == 6)  //帧尾校验正确
		//{
		//	EndNum = 0;
		//	EndNumFlag = false;
		//	ReceiveEndFlag = true;
		//	Serial.println("Get frame end... <Receive_LoRa_Cmd>");
		//	break;
		//}
	}

	if (ReceiveEndFlag)
	{
		iwdg_feed();
		Serial.println("Parsing LoRa command... <Receive_LoRa_Cmd>");
		Serial.flush();
		ReceiveEndFlag = false;

		if (FrameHeadDex != 0)  //第一个字节不是0xFE，说明有噪音干扰，重新从0xFE开始组合出一帧
		{
			unsigned char HeadStart = FrameHeadDex;
			for (unsigned char i = 0; i < (gReceiveLength - HeadStart); i++)
			{
				gReceiveCmd[i] = gReceiveCmd[FrameHeadDex++];
			}
		}

		if (gIsHandleMsgFlag)
		{
			Receive_Data_Analysis();//根据帧ID分析判断执行哪一个接收到的通用指令或私有指令
		}
		gReceiveLength = 0;
		iwdg_feed();
	}
	else
		gReceiveLength = 0;
}

/*
 @brief     : 根据验证接收的帧ID，决定返回对应的指令枚举
 @param     : 无
 @return    : frame id type(enum)
 */
Frame_ID Command_Analysis::FrameID_Analysis(void)
{
	unsigned int FrameID = ((gReceiveCmd[1] << 8) | gReceiveCmd[2]);
	switch (FrameID)
	{
	case 0xA000: return Modbus_Control;		break;//通用控制器modbus控制(A000)
	case 0xA001: return R_Modbus_Control;	break;//通用控制器modbus控制回执(A001)
	case 0xA002: return Output_default;		break;//设置输出默认状态及超时时间(A0002)
	case 0xA003: return Irrigation_Control; break;//服务器发送灌溉控制器控制指令(A003)
	case 0xA011: return Work_Para;			break;//基地服务器查询LoRa设备当前工作参数(A011)
	case 0xA012: return Set_Group_Num;		break;//基地服务器设置LoRa设备工作组编号(A012)
	case 0xA013: return SN_Area_Channel;	break;//基地服务器设置设备（主/子）SN及子设备总路数(A013)
	case 0xA014: return Work_Status;		break;//查询LoRa设备当前工作状态（A014）
	//case 0xA015: return Stop_Work;			break;
	//case 0xA020: return ResetRoll;			break;
	//case 0xA021: return Opening;			break;
	//case 0xA022: return Work_Limit;			break;

	default: memset(gReceiveCmd, 0x00, sizeof(gReceiveCmd));return Non_existent; break;
	}
}

/*
 @brief     : 验证接收到的LoRa数据里的CRC8校验码
 @param     : 1.验证数据的起始地址
			  2.验证数据的长度
 @return    : true or false.
 */
bool Command_Analysis::Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len)
{
	unsigned char ReceiveCRC8 = GetCrc8(&gReceiveCmd[verify_data_base_addr], verify_data_len);
	if (ReceiveCRC8 == gReceiveCmd[gReceiveLength - 7])
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的设备ID与本机是否相同
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Device_Type_ID(void)
{
	unsigned int DeviceTypeID = ((gReceiveCmd[4] << 8) | gReceiveCmd[5]);
	if (DeviceTypeID == 0x5555)
		return true;

	if (DeviceTypeID == DEVICE_TYPE_ID)
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的指令是否是群发指令
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Verify_Mass_Commands(void)
{
	gReceiveCmd[6] == 0x55 ? gMassCommandFlag = true : gMassCommandFlag = false;
}

/*
 @brief   : 验证接收的区域号与本地是否相同
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Area_Number(void)
{
	if (gReceiveCmd[7] == 0x55) return true;  //0x55:群控指令，忽略区域号

	unsigned char LocalAreaNumber = SN.Read_Area_Number();
	if (gReceiveCmd[7] == LocalAreaNumber || LocalAreaNumber == 0)
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的工作组号是否在本机组控列表内。
			如果接收的组号是0x55，表明此指令忽略组控，发送给区域内所有的设备
			如果本设备还未申请注册过服务器，不用校验组号。
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Work_Group(void)
{
	if (gReceiveCmd[8] == 0x55) return true;  //0x55:群控指令，忽略工作组号

	unsigned char LocalGroupNumber[5], ReceiveGroupSingleNumber = gReceiveCmd[8];
	unsigned char UndefinedGroupNum = 0;
	SN.Read_Group_Number(&LocalGroupNumber[0]);

	for (unsigned char i = 0; i < 5; i++)
	{
		if (ReceiveGroupSingleNumber == LocalGroupNumber[i]) return true;

		/*全为0，说明是未初始化的组号，算校验通过*/
		if (LocalGroupNumber[i] == 0x00)
		{
			UndefinedGroupNum++;
			if (UndefinedGroupNum == 5)
				return true;
		}
	}
	return false;
}

/*
 @brief   : 验证接收的指令CRC8校验、设备类型码、区域号、工作组是否合法。
			可以通过形参决定是否屏蔽验证区域号和工作组。
 @param   : 1.验证的数据起始地址
			2.验证的数据长度
			3.是否要验证区域号标志位
			4.是否要验证工作组号标志位
 @return  : true or false.
 */
bool Command_Analysis::Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag = true, bool group_flag = true)
{
	if ((Verify_CRC8(verify_data_base_addr, verify_data_len) == true) || ((gReceiveCmd[gReceiveCmd[3]+11-7]) == 0xD6))// (gReceiveCmd[gReceiveLength - 7] == 0xD6)
	{
		if (Verify_Device_Type_ID() == true)
		{
			Verify_Mass_Commands();
			if (Verify_Area_Number() == true || area_flag == false)
			{
				if (Verify_Work_Group() == true || group_flag == false)
					return true;
				else
				{
					Serial.println("Not this device group number... <Verify_Frame_Validity>");
					Serial.println("不是这个设备组号... <Verify_Frame_Validity>");
				}
			}
			else
			{
				Serial.println("Not this device area number... <Verify_Frame_Validity>");
				Serial.println("不是这个设备区域号... <Verify_Frame_Validity>");
			}
		}
		else
		{
			Serial.println("Device type ID ERROR! <Verify_Frame_Validity>");
			Serial.println("设备类型ID错误! <Verify_Frame_Validity>");
		}
	}
	else
	{
		Serial.println("CRC8 ERROR! <Verify_Frame_Validity>");
	}
	return false;
}

/*
 @brief   : 根据帧ID分析判断执行哪一个接收到的通用指令或私有指令
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Receive_Data_Analysis(void)
{
	switch (FrameID_Analysis())//根据验证接收的帧ID，决定返回对应的指令枚举
	{
	/*通用指令*/
	case Work_Para: Query_Current_Work_Param();		break;//服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等(A011)
	case Set_Group_Num: Set_Group_Number();			break;//设置本设备的工作组号，不需要验证本设备原有工作组号(A012)
	case SN_Area_Channel: Set_SN_Area_Channel();	break;//基地服务器设置设备（主/子）SN及子设备总路数(A013)
	case Work_Status: Detailed_Work_Status();		break;//查询本设备详细工作状态(A014)
	// case Stop_Work: Stop_Work_Command();			break;//强制停止当前设备的工作(A015)
	case Non_existent: Serial.println("不存在于本设备帧ID!!!");break;//异常处理
	///*卷膜机私有指令*/
	//case ResetRoll: ResetRoll_Command();			break;//重置卷膜测量行程(A020)
	//case Opening: Opening_Command();				break;//设置卷膜开度(A021)
	//case Work_Limit: Working_Limit_Command();		break;//电机工作电压阈值、上报状态间隔值设置(A022)

	/*通用控制器私有指令*/
	case Modbus_Control: General_controller_control_command();		break;//服务器发送通用控制器Modbus控制指令(A000)
	case R_Modbus_Control: R_General_controller_control_command();  break;//服务器发送通用控制器Modbus控制指令接收回执(A001)
	case Output_default: Set_General_controller_output_init();		break;//服务器设置通用控制器输出默状态(A002)

	/*12路灌溉控制器私有指令*/
	case Irrigation_Control: Irrigation_Controllor_control_command(); break;//服务器发送灌溉控制器控制指令(A003)
	}
}

/*
 @brief   : 服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等。（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Query_Current_Work_Param(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 | 申号标志 |  查询角色 | 采集时间间隔      |  时间   |  预留位     |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number | intent   |  channel | collect interval  |  RTC   |   allocate  |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte       1 byte       1 byte      1 byte      2 byte           7 byte      8 byte     1 byte      6 byte

	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

	if (Verify_Frame_Validity(4, 23, true, false) == true)
	{
		Serial.println("A011 <Query_Current_Work_Param>");
		Serial.flush();
		if (gReceiveCmd[8] == 0x01) //配置参数标志（LoRa大棚传感器是0x00配置采集时间）
		{
			Get_receipt = true;
			/* 预留位第一字节用来设置LoRa的通信模式 */
			if (LoRa_Para_Config.Save_LoRa_Com_Mode(gReceiveCmd[19]))
			{
				Message_Receipt.General_Receipt(SetLoRaModeOk, 1);
				LoRa_MHL9LF.Parameter_Init(true);
				//Message_Receipt.Working_Parameter_Receipt(true, 2);
			}
			else
			{
				Message_Receipt.General_Receipt(SetLoRaModeErr, 2);
				Serial.println("Set LoRa Mode Err设置LORA模式错误! <Query_Current_Work_Param>");
			}
		}
		else  //回执状态标志
		{
			Message_Receipt.Report_General_Parameter();
		}
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 设置本设备的工作组号，不需要验证本设备原有工作组号（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Set_Group_Number(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 | 工作组号   | 设备路数 |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag   |  Area number |  workgroup |  channel |   CRC8 |  |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte        5 byte       1 byte    1 byte      6 byte

	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

	if (Verify_Frame_Validity(4, 10, true, false) == true)
	{
		Serial.println("A012 <Set_Group_Number>");
		Serial.flush();
		Get_receipt = true;
		if (SN.Save_Group_Number(&gReceiveCmd[8]) == true)
		{
			Serial.println("Save group number success保存组号成功... <Set_Group_Number>");
			Message_Receipt.General_Receipt(AssignGroupIdArrayOk, 2);
		}
		else
		{
			Serial.println("Save group number failed保存组号失败 !!! <Set_Group_Number>");
			/*  */
			Message_Receipt.General_Receipt(AssignGroupIdArrayErr, 1);
		}
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 设置本设备的SN码、区域号、设备路数等参数（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Set_SN_Area_Channel(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   | 所在执行区域号 |  设备路数      |  子设备总路数           |  SN码       | 校验码   |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag   |   Area number | Device channel |  subordinate channel   | SN code     |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte          1 byte           1 byte                9 byte       1 byte      6 byte

	if (Verify_Frame_Validity(4, gReceiveCmd[3], false, false) == true)
	{
		Serial.println("A013 <Set_SN_Area_Channel>");
		Serial.flush();
		Get_receipt = true;
		if (SN.Save_SN_Code(&gReceiveCmd[10]) == true && SN.Save_BKP_SN_Code(&gReceiveCmd[10]) == true)
		{
			Serial.println("Set SN code success... <Set_SN_Area_Channel>");
			if (SN.Save_Area_Number(gReceiveCmd[7]) == true)
			{
				Serial.println("Save area number success保存区域ID成功... <Set_SN_Area_Channel>");
				Message_Receipt.General_Receipt(SetSnAndSlaverCountOk, 1);
				SN.Set_SN_Access_Network_Flag();
			}
			else
			{
				Serial.println("Save area number ERROR保存区域ID失败 !!! <Set_SN_Area_Channel>");
				/*  */
				Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
			}
		}
		else
		{
			Serial.println("Save SN code ERROR保存SN出错 !!! <Set_SN_Area_Channel>");
			/*  */
			Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
		}
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 查询本设备详细工作状态（服务器 ---> 本设备）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Detailed_Work_Status(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号	| 组ID		|  设备路数 |随机值	|校验码  | 帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number	| GroupId	|  channel |randomID|   CRC8 |Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte        1 byte		| 1 byte	|    1 byte|2 byte	|   1 byte| 6 byte

	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true) 
	{
		Serial.println("A014 <Detailed_Work_Status>");
		Serial.flush();
		Get_receipt = false;//每次进入至该函数，将该值置为false

		Serial.println("查询本设备详细工作状态 <Detailed_Work_Status>");
		//Message_Receipt.Working_Parameter_Receipt(true, 1);

		/*这里存时间间隔*/
		unsigned int E014Interval = gReceiveCmd[10] * 0x100 + gReceiveCmd[11];
		if (E014Interval < 5)
		{
			Serial.println("E014Interval值过小!!!强制置为5");
			E014Interval = 5;
			gReceiveCmd[10] = 0x00;gReceiveCmd[11] = 0x05;
		}
		Serial.println(String("E014Interval = ") + E014Interval);
		if (!InitState.Save_E014Interval(gReceiveCmd[10], gReceiveCmd[11]))
		{
			Serial.println("保存E014Interval失败!!! <General_controller_control_command>");
		}

		Message_Receipt.Working_Parameter_Receipt(false, 1, gReceiveCmd[12], gReceiveCmd[13]);
	}

	memset(gReceiveCmd, 0x00, gReceiveLength);
}

 /*
  @brief   : 强制停止当前设备的工作（服务器 ---> 本设备）
  @param   : 无
  @return  : 无
  */
 void Command_Analysis::Stop_Work_Command(void)
 {
 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 |  设备路数 |  校验码  |     帧尾 
 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number |   channel |   CRC8 |  |  Frame end
 	//  1 byte       2 byte      1 byte          2 byte        1 byte        1 byte         1 byte    1 byte      6 byte

 	//if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

 	//if (Verify_Frame_Validity(4, 6, true, true) == true)
 	//{
 	//	//gStopWorkFlag = true;
 	//	//Message_Receipt.General_Receipt(TrunOffOk, 1);
 	//	//MANUAL_ROLL_ON;  //使能手动
 	//}
 	//memset(gReceiveCmd, 0x00, gReceiveLength);
 }

// /*
//  @brief     : 重置卷膜测量行程
//  @param     : 无
//  @return    : 无
//  */
// void Command_Analysis::ResetRoll_Command(void)
// {
// 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 |所在执行区域号 |  工作组号   | 设备路数 |  校验码  |     帧尾 
// 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag  |  Area number |   workgroup | channel |   CRC8 |  |  Frame end
// 	//  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte         1 byte      1 byte    1 byte      6 byte

// 	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

// 	if (Verify_Frame_Validity(4, 6, true, true) == true)
// 	{
// 		/*如果电机手动卷膜按键电路异常，禁止自动卷膜，等待更换设备*/
// 		if (gManualKeyExceptionFlag)
// 		{
// 			Serial.println("Manual roll key exception手动卷膜键异常 !!! <ResetRoll_Command>");
// 			return;
// 		}
// 		/*如果当前正在卷膜，不进行二次卷膜*/
// 		else if (gResetRollWorkingFlag == true || gOpeningWorkingFlag == true || gForceRollWorkingFlag == true)
// 		{
// 			Serial.println("The motor is resetting the distance卷膜机正在重置行程... <ResetRoll_Command>");
// 			return;
// 		}
// 		/*如果当前正在手动卷膜。拒绝执行自动卷膜*/
// 		else if (gManualUpDetectFlag || gManualDownDetectFlag)
// 		{
// 			if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
// 			if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
// 			if (gManualUpDetectFlag || gManualDownDetectFlag)
// 			{
// 				Serial.println("Detect manual rolling检测到手动卷膜... <ResetRoll_Command>");
// 				/*
// 				  *待处理事情
// 				*/
// 				return;
// 			}
// 		}
// 		else
// 		{
// 			MANUAL_ROLL_OFF;/*使能/失能手动卷膜*/
// 			detachInterrupt(DEC_MANUAL_DOWN_PIN);
// 			detachInterrupt(DEC_MANUAL_UP_PIN);
// 			Message_Receipt.General_Receipt(RestRollerOk, 1);
// 			Motor_Operation.Reset_Motor_Route();
// 			attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 			attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 			MANUAL_ROLL_ON;/*使能/失能手动卷膜*/
// 			iwdg_feed();
// 		}
// 	}
// 	memset(gReceiveCmd, 0x00, gReceiveLength);
// }

// /*
//  @brief     : 设置卷膜开度
//  @param     : 无
//  @return    : 无
//  */
// void Command_Analysis::Opening_Command(void)
// {
// 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 |  工作组号   | 设备路数 |  开度    | 校验码  |     帧尾 
// 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag  |  Area number |   workgroup | channel |  oepning | CRC8 |  |  Frame end
// 	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte    1 byte    1 byte      6 byte

// 	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

// 	if (Verify_Frame_Validity(4, 7, true, true) == true)  //如果校验通过（CRC校验、区域号校验、组号校验）
// 	{
// 		Serial.println("这是卷膜函数《测试》");
// 		/*如果电机手动卷膜按键电路异常，禁止自动卷膜*/
// 		if (gManualKeyExceptionFlag)
// 		{
// 			Set_Motor_Status(MANUAL_KEY_EXCEPTION);
// 			Message_Receipt.Working_Parameter_Receipt(false, 2);
// 			Serial.println("Manual roll key exception !!! <Opening_Command>");
// 			Serial.println("手动滚动键异常!!! <Opening_Command>");
// 			return;
// 		}
// 		/*如果当前正在手动卷膜。拒绝执行自动卷膜*/
// 		else if (gManualUpDetectFlag || gManualDownDetectFlag)
// 		{
// 			if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
// 			if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
// 			if (gManualUpDetectFlag || gManualDownDetectFlag)
// 			{
// 				Serial.println("Detect manual rolling... <Opening_Command>");
// 				Serial.println("检测手动卷膜... <Opening_Command>");
// 				/*
// 				  *待处理事情
// 				*/
// 				return;
// 			}
// 		}
// 		/*如果当前正在卷膜，不进行二次卷膜*/
// 		else if (gOpeningWorkingFlag == true || gResetRollWorkingFlag == true || gForceRollWorkingFlag == true)
// 		{
// 			Serial.println("Currently the motor is opening, and others opening cannot to do !!! <Opening_Command>");
// 			Serial.println("目前电机正在卷膜，不进行二次卷膜 !!! <Opening_Command>");
// 			return;
// 		}

// 		/*失能手动卷膜， 失能检测手动卷膜按键中断*/
// 		detachInterrupt(DEC_MANUAL_DOWN_PIN);
// 		detachInterrupt(DEC_MANUAL_UP_PIN);
// 		MANUAL_ROLL_OFF;

// 		Message_Receipt.General_Receipt(OpenRollerOk, 1); //通用回执，告诉服务器接收到了开度卷膜命令
// 		Serial.println("发送回执信息成功《测试》");

// 		volatile unsigned char opening_value = gReceiveCmd[10]; //获取从服务器接收的开度值

// 		/*
// 		  *如果是开度卷膜，且要求全关或全开。本次当前开度正好已经是全关或全开了
// 		  *那么就不需要再次进入到卷膜函数里。
// 		  *应用在于：假如冬天不能开棚，本来也确实关着。服务器发送一个关棚开度，
// 		  *程序会先判断有没有重置行程，假如有些因素导致还要重置行程，那么就会打开棚了。
// 		  *当然，不适用于强制卷膜
// 		  *还有，假如发来的开度是0或100，说明需要开棚和关棚，一般用于天冷了或天热了，这个时候就
// 		  *不去判断是否发来的和保存的是否一至了，而是发来的只要是关棚或开棚，同时需要重置行程，
// 		  *就会用强制开棚和强制关棚来操作，最大限度不去重置行程。
// 		  *而其他的开度值，就会去判断上一次的开度和本次的是否相等，如果相等，就不动作
// 		  *如果不相等，同时又需要重置行程，那么只能必须先重置行程才能开到某个开度了。
// 		 */
// 		if (opening_value >= 0 && opening_value <= 100)
// 		{
// 			unsigned char RealTimeOpenTemp = Roll_Operation.Read_RealTime_Opening_Value();//读取实时卷膜开度值

// 			if (!Roll_Operation.Read_Route_Save_Flag())
// 			{
// 				if (opening_value == 0 || opening_value == 100)
// 				{
// 					if (opening_value == 0)
// 						opening_value = 0xF0;
// 					else
// 						opening_value = 0xF1;

// 					Serial.println("Prepare Force Open or Close. Be careful... <Opening_Command>");
// 					Serial.println("准备强行全开或全关。小心... <Opening_Command>");
// 					Motor_Operation.Force_Open_or_Close(opening_value);
// 					memset(gReceiveCmd, 0x00, gReceiveLength);
// 					/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
// 					attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 					attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 					MANUAL_ROLL_ON;
// 					return;
// 				}
// 			}

// 			if (RealTimeOpenTemp == opening_value)
// 			{
// 				Serial.println("Film has been rolled to the current opening, do not repeat the film... <Opening_Command>");
// 				Serial.println("已滚动到当前开度值，不要重复卷膜…... <Opening_Command>");
// 				Set_Motor_Status(ROLL_OK);
// 				Message_Receipt.Working_Parameter_Receipt(true, 2);

// 				attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 				attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 				MANUAL_ROLL_ON;
// 				return;
// 			}
// 		}

// 		//正常开度值范围是0到100，如果是F0，表明是强制关棚，如果是F1，表明是强制开棚
// 		if (opening_value == 0xF0 || opening_value == 0xF1)
// 		{
// 			Serial.println("Prepare Force Open or Close. Be careful... <Opening_Command>");
// 			Serial.println("准备全开全关，请小心... <Opening_Command>");
// 			Motor_Operation.Force_Open_or_Close(opening_value);
// 			memset(gReceiveCmd, 0x00, gReceiveLength);
// 			/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
// 			attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 			attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 			MANUAL_ROLL_ON;
// 			return;
// 		}

// 		if (Roll_Operation.Read_Route_Save_Flag())  //如果已经重置行程过
// 		{
// 			if (Roll_Operation.Save_Current_Opening_Value(opening_value))  //保存当前开度值
// 			{
// 				Serial.println("Begin to coiling... <Opening_Command>");
// 				Serial.println("开始卷膜... <Opening_Command>");
// 				Motor_Operation.Motor_Coiling();  //开始卷膜
// 				iwdg_feed();
// 			}
// 			else  //保存开度值异常
// 			{
// 				Serial.println("Save current opening value ERROR !!! <Opening_Command>");
// 				Serial.println("保存当前开度值错误!!! <Opening_Command>");
// 				Set_Motor_Status(STORE_EXCEPTION);
// 				Message_Receipt.Working_Parameter_Receipt(false, 2);
// 			}
// 		}
// 		else  //或者没有重置行程
// 		{
// 			Serial.println("The film has not measured the distance, first measure, then roll the film... <Opening_Command>");
// 			Serial.println("电机未测量距离，先测量距离，再卷膜... <Opening_Command>");
// 			if (Motor_Operation.Reset_Motor_Route() == true) //先重置行程，再开度卷膜
// 			{
// 				Serial.println("Roll OK, motor begin coiling... <Opening_Command>");
// 				Serial.println("重置行程完成,电机开始转动... <Opening_Command>");

// 				if (Roll_Operation.Save_Current_Opening_Value(opening_value))  //保存当前开度值
// 				{
// 					gAdjustOpeningFlag = false;
// 					Motor_Operation.Motor_Coiling();  //开始卷膜
// 				}
// 				else  //保存开度值操作异常
// 				{
// 					Serial.println("Save current opening value ERROR !!! <Opening_Command>");
// 					Serial.println("保存当前开度值错误 !!! <Opening_Command>");
// 					Set_Motor_Status(STORE_EXCEPTION);
// 					Message_Receipt.Working_Parameter_Receipt(false, 2);
// 				}
// 			}
// 			else  //重置行程失败
// 				Serial.println("Reset motor route failed !!! <Opening_Command>");
// 			iwdg_feed();
// 		}
// 		/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
// 		attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 		attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 		MANUAL_ROLL_ON;
// 	}
// 	memset(gReceiveCmd, 0x00, gReceiveLength);
// }

// /*
//  @brief     : 电机工作电压阈值、上报状态间隔值设置（网关 ---> 本机）
//  @param     : 无
//  @return    : 无
//  */
// void Command_Analysis::Working_Limit_Command(void)
// {
// 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  | 所在执行区域号 |  工作组号   | 设备路数 | 低电压阈值       |   高电压阈值      | 状态上报间隔     |校验码 | 帧尾 
// 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |   Area number |   workgroup | channel | LowVolThreshold | HighVolThreshold |  ReprotInterval | CRC8 |  Frame end
// 	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte     2 byte             2 byte              1 byte        1 byte  6 byte

// 	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

// 	if (Verify_Frame_Validity(4, 11, true, true) == true)
// 	{
// 		if (Roll_Operation.Save_Roll_Work_Voltage_and_Report_Interval(&gReceiveCmd[10]) == true)
// 			Message_Receipt.General_Receipt(LimitRollerOk, 1);
// 		else
// 		{
// 			Serial.println("Save working threshold ERROR !");
// 			Serial.println("保存工作阈值错误!");
// 			Message_Receipt.General_Receipt(LimitRollerErr, 1);
// 			Set_Motor_Status(STORE_EXCEPTION);
// 			Message_Receipt.Working_Parameter_Receipt(false, 2);
// 		}
// 	}
// 	memset(gReceiveCmd, 0x00, gReceiveLength);
//}

 /*
  @brief     : 服务器发送通用控制器Modbus控制指令（网关 ---> 本机）
  @param     : 无
  @return    : 无
  */
void Command_Analysis::General_controller_control_command(void)
{
	  /*|字节索引	|  0		| 1 - 2		| 3			| 4	-5			| 6				| 7			| 8			| 9 - 10	| 11 - 12   | 13 - n		|		|               |
		|数据域		| frameHead	| frameId	| dataLen	| DeviceTypeId	| isBroadcast	| zoneId	| groupId	| randomId	| interval	| modbusPacket	| CRC8	| frameEnd      |
		|长度(byte)	| 1			| 2			| 1			| 2				| 1				| 1			| 1			| 2			| 2			| n				| 1		| 2				|
		|示例数据	| FE		| A000		| 9+n		| C003			| 00			| 01		| 01		| 1234		| 0000		| XXXXXXXX		| 00	| 0D0A0D0A0D0A	|*/
		//备注：A000的interval用来指E000的上报时间间隔

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)
	{
		Serial.println("A000 <General_controller_control_command>");
		Serial.flush();
		Get_receipt = false;//每次进入至该函数，现将该值置为false
		randomId_1 = gReceiveCmd[9]; randomId_2 = gReceiveCmd[10];//

		Serial.println("服务器发送通用控制器Modbus控制指令 <General_controller_control_command>");

		unsigned int E000interval = gReceiveCmd[11] * 0x100 + gReceiveCmd[12];
		Serial.println(String("E000interval = ") + E000interval);

		if (!InitState.Save_E000Interval(gReceiveCmd[11], gReceiveCmd[12]))
		{
			Serial.println("保存E000interval失败!!! <General_controller_control_command>");
			gStatus_E014 = Failed_save_E000interval;//保存E000interval失败

			/*得到随机值*/
			unsigned char random_1 = random(0, 255);
			unsigned char random_2 = random(0, 255);
			Serial.println(String("random_1 = ") + String(random_1, HEX));
			Serial.println(String("random_2 = ") + String(random_2, HEX));

			/*这里上报实时状态*/
			//Message_Receipt.Working_Parameter_Receipt(false, 1, random_1, random_2);
		}

		unsigned char modbusPacket[20] = { 0x00 };unsigned int modbusPacket_Length = gReceiveCmd[3] - 9;
		// unsigned char R_modbusPacket[20] = { 0x00 };unsigned int R_modbusPacket_Length = 0;
		if (modbusPacket_Length > 20)
		{
			//此处应该有异常处理！！！
			Serial.println("modbusPacket_Length超过数组预设值!!! <General_controller_control_command>");
			gStatus_E014 = Modbuspacket_length_overflow;//modbusPacket_Length超过数组预设值

			/*得到随机值*/
			unsigned char random_1 = random(0, 255);
			unsigned char random_2 = random(0, 255);
			Serial.println(String("random_1 = ") + String(random_1, HEX));
			Serial.println(String("random_2 = ") + String(random_2, HEX));

			/*这里上报实时状态*/
			//Message_Receipt.Working_Parameter_Receipt(false, 1, random_1, random_2);
		}
		else
		{
			for (size_t i = 0; i < modbusPacket_Length; i++)
			{
				modbusPacket[i] = gReceiveCmd[13 + i];
				//Serial.println(modbusPacket[i], HEX);
			}
			gStatus_E014 = A000_Received_Success;//A000指令接收成功

			/*得到随机值*/
			unsigned char random_1 = random(0, 255);
			unsigned char random_2 = random(0, 255);
			Serial.println(String("random_1 = ") + String(random_1, HEX));
			Serial.println(String("random_2 = ") + String(random_2, HEX));

			/*这里上报实时状态*/
			//Message_Receipt.Working_Parameter_Receipt(false, 1, random_1, random_2);
		}

		Modbus_Coil.Modbus_Realization(modbusPacket, modbusPacket_Length);//设置输出线圈状态，modbus实现

		Message_Receipt.Control_command_Receipt(gReceiveCmd[11], gReceiveCmd[12], 1, gReceiveCmd[9], gReceiveCmd[10]);
		DOStatus_Change = true;//接收到指令后上报实时状态
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

 /*
  @brief     : 服务器发送通用控制器Modbus控制指令接收回执（网关 ---> 本机）
  @param     : 无
  @return    : 无
  */
void Command_Analysis::R_General_controller_control_command(void)
{
  /*| 字节索引	| 0			| 1 - 2		| 3			| 4 - 5			| 6				| 7			| 8			| 9 - 10	| 11 - 12	| 13-19				| 20	| 21-28		| 20	| 21-26			|
	| 数据域	| frameHead | frameId	| dataLen	| DeviceTypeId	| isBroadcast	| zoneId	| groupId	| random	| interval	| RTC				| Mode	| TF/RFREQ	| CRC8	| frameEnd		|
	| 长度		| 1			| 2			| 1			| 2				| 1				| 1			| 1			| 2			| 2			| 7					| 1		| 8			| 1		| 6				|
	| 示例数据	| FE		| A001		| 09		| C003			| 00			| 01		| 01		| 1234		| 0000		| 20200225133601	| F1	| XXXXXXXX	| 00	| 0D0A0D0A0D0A	|*/
	//备注：A001的interval用来指E014的上报时间间隔

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Serial.println("A001 <R_General_controller_control_command>");
		Serial.flush();

		if (gLoRaCSQ[0] == 0 || gLoRaCSQ[1] == 0)
		{
			Serial.println("开始查询信号质量");
			LoRa_MHL9LF.LoRa_AT(gLoRaCSQ, true, AT_CSQ_, 0);
		}

		if ((gReceiveCmd[9] == randomId_1) && (gReceiveCmd[10] == randomId_2))
		{
			Get_receipt = true;
			Serial.println("服务器发送通用控制器Modbus控制指令回执 <R_General_controller_control_command>");
			unsigned int E014Auto_report = gReceiveCmd[11] * 0x100 + gReceiveCmd[12];
			Serial.println(String("E014Auto_report = ") + E014Auto_report);
			if (E014Auto_report < 180)
			{
				Serial.println("E014Auto_report的值过低,自动修订为180 <R_General_controller_control_command>");
				gReceiveCmd[11] = 0x00;gReceiveCmd[12] = 0xB4;
			}
			
			if (!InitState.Save_E014Auto_report(gReceiveCmd[11], gReceiveCmd[12]))
			{
				Serial.println("保存E014Auto_report失败!!! <General_controller_control_command>");
			}

			unsigned char RTC[7];//
			for (size_t i = 0; i < 7; i++)
			{
				RTC[i] = gReceiveCmd[13 + i];
			}
			Private_RTC.Update_RTC(&RTC[0]);

			// /* 预留位第一字节用来设置LoRa的通信模式 */
			// if(LoRa_Para_Config.Save_LoRa_Com_Mode(gReceiveCmd[20]))
			// {
			// 	Message_Receipt.General_Receipt(SetLoRaModeOk, 1);
			// 	LoRa_MHL9LF.Parameter_Init(true);
			// 	// Message_Receipt.Working_Parameter_Receipt(true, 2);
			// }
			// else 
			// {
			// 	Message_Receipt.General_Receipt(SetLoRaModeErr, 2);
			// 	Serial.println("Set LoRa Mode Err! <Query_Current_Work_Param>");
			// }

			// /* 前4位为发送频点，后4位为接收频点，若是全为0则表示不更改 */
			// if (!(gReceiveCmd[21] == 0x00 && gReceiveCmd[22] == 0x00 && gReceiveCmd[23] == 0x00 && gReceiveCmd[24] == 0x00))
			// {
			// 	String TFREQ = String(gReceiveCmd[21],HEX)+String(gReceiveCmd[22],HEX)+String(gReceiveCmd[23],HEX)+String(gReceiveCmd[24],HEX);
			// 	Serial.println(String("TFREQ = ") + TFREQ);
			// 	// if (LoRa_MHL9LF.Param_Check(AT_TFREQ_, "1C578DE0", false))
			// 	// {
			// 	// 	// Message_Receipt.Working_Parameter_Receipt(true, 2);
			// 	// }
			// 	// else
			// 	// {
			// 	// 	Message_Receipt.General_Receipt(SetLoRaTFREQErr, 2);
			// 	// 	Serial.println("Set LoRa TFREQ Err! <Query_Current_Work_Param>");
			// 	// }
			// }
			// /* 前4位为发送频点，后4位为接收频点，若是全为0则表示不更改 */
			// if (!(gReceiveCmd[24] == 0x00 && gReceiveCmd[25] == 0x00 && gReceiveCmd[26] == 0x00 && gReceiveCmd[27] == 0x00))
			// {
			// 	String RFREQ = String(gReceiveCmd[24],HEX)+String(gReceiveCmd[25],HEX)+String(gReceiveCmd[26],HEX)+String(gReceiveCmd[27],HEX);
			// 	Serial.println(String("RFREQ = ") + RFREQ);
			// 	// if (LoRa_MHL9LF.Param_Check(AT_RFREQ_, "1C578DE0", false))
			// 	// {
			// 	// 	// Message_Receipt.Working_Parameter_Receipt(true, 2);
			// 	// }
			// 	// else
			// 	// {
			// 	// 	Message_Receipt.General_Receipt(SetLoRaTFREQErr, 2);
			// 	// 	Serial.println("Set LoRa TFREQ Err! <Query_Current_Work_Param>");
			// 	// }
			// }

			return;
		}
		
	}
	Get_receipt = false;
}


/*
 @brief     : 服务器设置通用控制器输出默认状态（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Set_General_controller_output_init(void)
{
	/*	| 字节索引		| 0			| 1-2		| 3			|4-5			|6			|7		|8-9		|	10-17	| 18-33		| 34-35		| 36-N	|		|				|
		| 数据域		| frameHead | frameId	| dataLen	|DeviceTypeId	|IsBroadcast|ZoneId	|randomId	| DOInit	| AOInit	| timeout	| RS485 | CRC8	| frameEnd		|
		| 长度（byte）	| 1			| 2			| 1			|2				|1			|1		|2			| 8			| 16		| 2			| n		| 1		| 6				|
		| 示例数据		| FE		| A002		| 26		|C003			|00			|01		|1234		| FFFFFF..	|FFFF...	| 012C		|       | 00	| 0D0A0D0A0D0A	|*/

	/*	|Status_E002:	|0x00	|0x01			|0x02			|0x03		|0x04		|
		|				|正常	|保存超时时间失败	|保存初始化值失败	|485无回执	|485回执溢出	|*/

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Serial.println("A002 <Set_General_controller_output_init>");
		Serial.flush();
		Get_receipt = true;
		unsigned char Status_E002 = 0x00;//状态

		if (!InitState.Save_Timeout(gReceiveCmd[34], gReceiveCmd[35]))
		{
			Serial.println("保存timeout失败!!!<Set_General_controller_output_init>");
			Status_E002 = 0x01;
		}
		Serial.println("保存timeout成功<Set_General_controller_output_init>");

		//Serial.println("开始保存通用控制器输出默认状态<Set_General_controller_output_default>");
		//Serial.println(String(gReceiveCmd[8])+String(gReceiveCmd[9]));

		unsigned char DO_Init[8] = { 0x22 };unsigned char AO_Init[16] = { 0x33 };

		//将DO初始化的值赋给DO_Init[8]
		for (size_t i = 0; i < 8; i++)
		{
			DO_Init[i] = gReceiveCmd[i + 10];
			//Serial.println(String("DO_Init[") + i + "]=" + DO_Init[i]);
		}

		//将AO初始化的值赋给AO_Init[8]
		for (size_t i = 0; i < 16; i++)
		{
			AO_Init[i] = gReceiveCmd[i + 18];
			//Serial.println(String("AO_Init[") + i + "]=" + AO_Init[i]);
		}
		
		if (InitState.Save_DO_InitState(DO_Init) && InitState.Save_AO_InitState(AO_Init))//读取初始状态的标志位
		{
			Serial.println("保存初始化值成功<Set_General_controller_output_init>");
			InitState.Save_InitState_Flag();//存储初始状态的标志位
			Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
		}
		else
		{
			Serial.println("保存初始化值失败<Set_General_controller_output_init>");
			Status_E002 = 0x02;
			InitState.Clean_InitState_Flag();//清除初始状态的标志位
			Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
		}

		unsigned char Modbus_Instructions[10]; unsigned char Modbus_Length = gReceiveCmd[3] - 32;
		Serial.println("-----");
		if (Modbus_Length <= 10)
		{
			for (size_t i = 0; i < Modbus_Length; i++)
			{
				Modbus_Instructions[i] = gReceiveCmd[36 + i];
				//Serial.println(Modbus_Instructions[i], HEX);
			}
			Serial.println("-----");
			//Serial.write(&Modbus_Instructions[0], Modbus_Length);
			Serial2.write(Modbus_Instructions, Modbus_Length);
		}
		
		iwdg_feed();
		delay(1000);//等待回执
		unsigned char R_Modbus_Instructions[20]; unsigned int R_Modbus_Length = 0;
		bool Overflow_485 = false;
		//unsigned char ceshi[8] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08 };
		while (Serial2.available() > 0)
		{
			R_Modbus_Instructions[R_Modbus_Length++] = Serial2.read();
			if (R_Modbus_Length > 10)
			{
				Serial.println("R_Modbus_Length > 10");
				R_Modbus_Length = 0;
				Overflow_485 = true;
				break;
			}
		}
		/*for (size_t i = 0; i < 8; i++)
		{
			R_Modbus_Instructions[R_Modbus_Length++] =ceshi[i];
		}*/

		if (R_Modbus_Length > 0)
		{
			Serial.println(String("R_Modbus_Length = ") + R_Modbus_Length);
			for (size_t i = 0; i < R_Modbus_Length; i++)
			{
				Serial.println(R_Modbus_Instructions[i],HEX);
			}
		}
		else
		{
			Status_E002 = 0x03;
			Serial.println("485设备无回执 <Set_General_controller_output_init>");
			if (Overflow_485)
			{
				Serial.println("485设备回执溢出 <Set_General_controller_output_init>");
				Status_E002 = 0x04;
			}
		}

		Message_Receipt.Output_init_Receipt(Status_E002, 1, gReceiveCmd[8], gReceiveCmd[9], R_Modbus_Instructions, R_Modbus_Length);

		/*if (Roll_Operation.Save_Roll_Work_Voltage_and_Report_Interval(&gReceiveCmd[10]) == true)
			Message_Receipt.General_Receipt(LimitRollerOk, 1);
		else
		{
			Serial.println("Save working threshold ERROR !");
			Message_Receipt.General_Receipt(LimitRollerErr, 1);
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
		}*/
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器发送灌溉控制器控制指令(A025)（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Irrigation_Controllor_control_command(void)
{
	  /*| 字节索引	| 0			| 1 - 2		| 3			| 4 - 5			| 6				| 7			| 8-9		| 10 - 41	| 42 - 71	| 72-73		| 74-75	| 76-107	| 108	| 109 - 114     |
		| 数据域	| FrameHead | FrameId	| DataLen	| DeviceTypeId	| IsBroadcast	| zoneId	| randomId	| openSec	| interval	| timeout	| DOUsed| retryCnt	| CRC8	| FrameEnd      |
		| 长度		| 1			| 2			| 1			| 2				| 1				| 1			| 2			| 32		| 30		| 2			| 2		| 32		| 1		| 6				|
		| 示例数据	| FE		| A003		| 0x68(104)	| C003			| 00			| 01		| 1234		| 0005		| 0003		| 001e		| FF00	| 0005		| D6	| 0D0A0D0A0D0A	|*/
	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Serial.println("A003 <Irrigation_Controllor_control_command>");
		Serial.flush();
		Get_receipt = true;
		iwdg_feed();
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

		KeyDI7_num = 0;//将按键按下次数清零，就可以让E014一直上报
		GET_DO_ON = 0;//开启的数量

		//Get_receipt = true;

		#if WirlessSwitch_V1
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
		DOStatus_Change = true;


		if (InitState.Save_CyclicInterval(gReceiveCmd[72], gReceiveCmd[73]))
		{
			Serial.println("设置并保存参数成功 <Irrigation_Controllor_control_command>");
		}
		else
		{
			Serial.println("设置并保存参数失败!!! <Irrigation_Controllor_control_command>");
		}
		Cyclic_interval = InitState.Read_CyclicInterval();
		
		//DO_Num = gReceiveCmd[16];
		//retryCnt = gReceiveCmd[76] * 0x100 + gReceiveCmd[77];//得到循环次数
		/*if (DO_Num <= 0 || DO_Num > 8)
		{
			Serial.println("ERROR!!!一次性开启的DO数量错误");
			DO_Num = 1;
		}*/
		//Serial.println(String("开启时间openSec = ") + OpenSec);
		//Serial.println(String("单个间隔时间DO_Interval = ") + DO_Interval);
		Serial.println(String("循环间隔时间Cyclic_interval = ") + Cyclic_interval);
		//Serial.println(String("一次开启的DO数量DO_Num = ") + DO_Num);
		//Serial.println(String("循环次数retryCnt = ") + retryCnt);
		iwdg_feed();
		byte bitn = 0;
		word Outputs = 12; word UseOutputs = 12;
		word x;
		byte Bitread = 0;
		for (size_t i = 0; i < 12; i++)
		{
			x = (Outputs - UseOutputs) / 8;
			Bitread = ((gReceiveCmd[74 + x] >> bitn) & 0x01);
			//Serial.println(String("Bitread = ") + Bitread);
			if (Bitread)
			{
				//Need_Num++;
				Worktime[i] = gReceiveCmd[(2 * i) + 10] * 0x100 + gReceiveCmd[(2 * i) + 11];
				WorkInterval[i] = gReceiveCmd[(2 * i) + 42] * 0x100 + gReceiveCmd[(2 * i) + 43];
				retryCnt[i] = gReceiveCmd[(2 * i) + 76] * 0x100 + gReceiveCmd[(2 * i) + 77];
				/*Worktime_backups[i] = Worktime[i];
				WorkInterval_backups[i] = WorkInterval[i];*/
			}

			bitn++;
			if (bitn == 8) bitn = 0;

			UseOutputs--;
		}

		//Serial.println(String("需要开启的个数Need_Num = ") + Need_Num);

		
		/*if (Need_Num % DO_Num == 0)
		{
			fornum = Need_Num / DO_Num;
			Last_full = true;
			Serial.println("Last_full = true");
		}
		else
		{
			fornum = (Need_Num / DO_Num)+1;
			Last_full = false;
			Last_num = Need_Num % DO_Num;
			Serial.println("Last_full = false");
			Serial.println(String("Last_num = ") + Last_num);
		}
		fornum_backups = fornum;
		Serial.println(String("一轮循环需要的循环次数fornum = ") + fornum);*/

		/*for (size_t i = 0; i < 12; i++)
		{
			Serial.println(String("Worktime[") + i + "]= " + Worktime[i]);
			Serial.println(String("WorkInterval[") + i + "]= " + WorkInterval[i]);
			Serial.println(String("retryCnt[") + i + "]= " + retryCnt[i]);
			Serial.println("---");
		}*/
		Serial.flush();

		Message_Receipt.Irrigation_control_Receipt(1, gReceiveCmd);
		Start_Timer4();
		
		#elif WirlessSwitch_V2
		Serial.println("");
		
		#endif		
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}




 