 #include "main.h"  
 
 
 //�ؽڵ��CAN���ݽṹ
CAN1_Data_TypeDef  Joint_LF_U,//��ǰ�ϽǶȵ��1
                   Joint_LF_D,//��ǰ�½Ƕȵ��2
									 Joint_LB_U,//����ϽǶȵ��3
                   Joint_LB_D,//����½Ƕȵ��4
									 Joint_RB_U,//�Һ��ϽǶȵ��5
                   Joint_RB_D,//�Һ��½Ƕȵ��6
									 Joint_RF_U,//��ǰ�ϽǶȵ��7
                   Joint_RF_D;//��ǰ�½Ƕȵ��8


//�ؽڵ������
 char CAN_DATA_CMD_ON[8]    =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC};
 char CAN_DATA_CMD_OFF[8]   =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD};
 char CAN_DATA_CMD_SET_0[8] =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00};
 char CAN_DATA_CMD_Change_ID[8] =   {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X06}; //���һλΪID��

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
/*********************************************��������ת������**********************************************************************/

 float bit8TObit32(uint8_t *change_info)
{
	union
	{
    float f;
		char  byte[4];
	}u32val;
    u32val.byte[0]=change_info[0];
    u32val.byte[1]=change_info[1];
    u32val.byte[2]=change_info[2];
    u32val.byte[3]=change_info[3];
	return u32val.f;
}

u8 bit32TObit8(int index_need,int bit32)
{
	union
	{
    int  f;
		u8  byte[4];
	}u32val;
   u32val.f = bit32;
	return u32val.byte[index_need];
}
short u8ToShort(char a[],char b)
{
	union
	{
		short i;
		char byte[2];
	}u16val;
	u16val.byte[0]=a[b];
	u16val.byte[1]=a[b+1];
	return u16val.i;
}

char shortTou8(char bit,short data)
{
	union
	{
		short i;
		char byte[2];
	}u16val;
	u16val.i=data;
	return u16val.byte[bit];
}

int8_t shortToint8(char bit,short data)
{
	union
	{
		short i;
		int8_t byte[2];
	}u16val;
	u16val.i=data;
	return u16val.byte[bit];
}
short int8ToShort(int8_t a[],char b)
{
	union
	{
		short i;
		int8_t byte[2];
	}u16val;
	u16val.byte[0]=a[b];
	u16val.byte[1]=a[b+1];
	return u16val.i;
}

char floatTou8(char bit,float data)
{
	union
	{
		float i;
		char byte[4];
	}u16val;
	u16val.i=data;
	return u16val.byte[bit];
}

int float_to_uint(float x, float x_min, float x_max, int bits)
	{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

 
 
 
 
 
 

		