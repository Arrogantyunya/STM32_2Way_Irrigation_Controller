#ifndef _COMMAND_ANALYSIS_H
#define _COMMAND_ANALYSIS_H

#include <Arduino.h>

#define ON						0x00	//
#define OFF						0x01	//

enum Frame_ID {
	Modbus_Control, R_Modbus_Control,Output_default,Work_Para, Set_Group_Num, SN_Area_Channel, Work_Status
	/* , ResetRoll, Opening, Work_Limit, Stop_Work  */, Irrigation_Control,Non_existent
};

class Command_Analysis {
public:
	void Receive_LoRa_Cmd(void);

private:
	Frame_ID FrameID_Analysis(void);
	bool Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len);
	bool Verify_Device_Type_ID(void);
	void Verify_Mass_Commands(void);
	bool Verify_Area_Number(void);
	bool Verify_Work_Group(void);
	bool Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag, bool group_flag);

private:
	void Receive_Data_Analysis(void);
	void Query_Current_Work_Param(void);//��������ѯ��ǰȺ����ز���������������SN�롢·���������顢��
	void Set_Group_Number(void);//���ñ��豸�Ĺ�����ţ�����Ҫ��֤���豸ԭ�й������
	void Set_SN_Area_Channel(void);//���ñ��豸��SN�롢����š��豸·���Ȳ���
	void Detailed_Work_Status(void);//��ѯ���豸��ϸ����״̬
	// void ResetRoll_Command(void);//���þ�Ĥ�����г�
	// void Opening_Command(void);//���þ�Ĥ����
	// void Working_Limit_Command(void);//���������ѹ��ֵ���ϱ�״̬���ֵ����
	void Stop_Work_Command(void);//ǿ��ֹͣ��ǰ�豸�Ĺ���
	void General_controller_control_command(void);//����������ͨ�ÿ�����Modbus����ָ��
	void R_General_controller_control_command(void);//����������ͨ�ÿ�����Modbus����ָ����ջ�ִ
	void Set_General_controller_output_init(void);//����������ͨ�ÿ��������Ĭ��״̬

	void Irrigation_Controllor_control_command(void);//���������͹�ȿ���������ָ��(A025)
};



/*Create command analysis project*/
extern Command_Analysis LoRa_Command_Analysis;

extern bool gAccessNetworkFlag;
// extern bool gStopWorkFlag;
extern bool gMassCommandFlag;
extern bool gIsHandleMsgFlag;

//
extern bool Get_receipt;

extern unsigned int Worktime[16]; //16������ʱ��
extern unsigned int Worktime_backups[16];//16������ʱ��
extern unsigned int WorkInterval[16];//16�����ʱ��
extern unsigned int WorkInterval_backups[16];//16�����ʱ��ı���
extern unsigned int retryCnt[16];//ѭ����������
extern unsigned int DO_WayOpentime[16];//16·DO�Ŀ�ʼʱ��
extern unsigned int DO_WayClosetime[16];//16·DO�Ĺر�ʱ��
extern unsigned int DO_WayIntervalBengin[16];//16·DO�ļ����ʼʱ��
extern unsigned int DO_WayIntervalEnd[16];//16·DO�ļ������ʱ��
extern bool DO_WayOpen[16];//16·DO�򿪵ı�־λ
extern bool DO_WayInterval[16];//16·DO�����־λ
extern bool DO_WayComplete[16];//16·DO��ɵı�־λ
extern bool Irrigation_use;//���ڹ�ȵı�־λ
extern bool DO_Set[16];//DO���õı�־λ
//extern unsigned int OpenSec;//������ʱ��
//extern unsigned int DO_Interval;//�����ļ��ʱ��
//extern unsigned int DO_Num;//һ���Կ�����DO����
extern unsigned char Need_Num;//һ��ѭ����Ҫ������DO����
extern unsigned char Complete_Num;//һ��ѭ����ɿ�����DO����
extern unsigned int Cyclic_interval;//ѭ�����ʱ��
//extern unsigned char fornum;//һ��ѭ����Ҫ��ѭ�������������翪4·��ÿ�ο�2·��fornum=2��
//extern unsigned char fornum_backups;//һ��ѭ����Ҫ��ѭ�������ı��ݣ������翪4·��ÿ�ο�2·��fornum=2��
//extern bool Last_full;//���һ��ѭ���Ƿ��ܿ����������翪5·��ÿ�ο�2·�����һ�ֿ�������
//extern unsigned char Last_num;//���һ��ѭ��������ʱ��Ҫ�����ĸ���

#endif
