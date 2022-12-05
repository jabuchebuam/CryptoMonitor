#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#define Simulator

#ifdef Simulator
extern "C"
{
//extern float value_control(float value1);
extern void program_run(void);
extern void send_alarm(void);
extern uint32_t GetVarRTC(void);
//extern float value_control(float value1);
extern uint32_t data[10];
extern uint8_t coin_val[10][10];
extern uint8_t Data_BTC[3];

extern uint32_t Data_BTC_old_value [13];
extern uint32_t Data_ETH_old_value [13];
extern uint32_t Data_LTC_old_value [13];
extern uint32_t Data_BCH_old_value [13];
extern uint32_t Data_SOL_old_value [13];
extern uint32_t Data_ADA_old_value [13];
extern uint32_t Data_LINK_old_value [13];
extern uint32_t Data_GNO_old_value [13];

}
#endif


Model::Model() : modelListener(0)
{

}

void Model::tick()
{

#ifdef Simulator
	program_run();
#endif
	//modelListener-> = value_control();

}

uint32_t Model::GetMinVal(uint8_t index)
{

	if(min[index]==0)
	{
		Get_Vect_Val(index);
		SetMinVal(index, vect_val[12]);
	}

	return min[index];
}

uint32_t Model::GetMaxVal(uint8_t index)
{
	if(max[index]==0)
	{
		Get_Vect_Val(index);
		SetMaxVal(index, vect_val[11]);
	}

	return max[index];
}

void Model::SetMinVal(uint8_t index, uint32_t val)
{
	min[index] = val;
}

void Model::SetMaxVal(uint8_t index, uint32_t val)
{
	max[index] = val;
}

uint32_t* Model::Get_Vect_Val(uint8_t index)
{
	float m, x1, x2, y1, y2, n, y;
	static uint8_t control;

	switch(index)
	{
	case 0:
	{

//		y1 = 0;
//		x1 = 85000;
//		y2 = 100;
//		x2 = 91000;
//		m = ((y2 - y1)/(x2-x1));
//		m = 0.016666;
//		y1 = m*x1 + n;
//		n = -(m*x1) + y1;
//		n= -800;
//		y = m*85000 + n;

		for(uint8_t i = 0; i<10;i++)
		{
			m = (float) (Data_BTC_old_value[i] * 0.016666 - 1416);
			vect_val[i] = (uint32_t) m;
		}

		if(Data_BTC_old_value[10]>1000 && Data_BTC_old_value[10]<150000)
			vect_val[10] = Data_BTC_old_value[10];
		if(Data_BTC_old_value[11]>1000 && Data_BTC_old_value[11]<150000)
			vect_val[11] = Data_BTC_old_value[11];
		if(Data_BTC_old_value[12]>1000 && Data_BTC_old_value[12]<150000)
			vect_val[12] = Data_BTC_old_value[12];

	}
	break;

	case 1:
	{

//		y1 = 0;
//		x1 = 6200;
//		y2 = 100;
//		x2 = 6900;
//		m = ((y2 - y1)/(x2-x1));
//		m = 0.1428;
//		y1 = m*x1 + n;
//		n = -(m*x1) + y1;
//		n= -483.33;
//		y = m*6500 + n;

		for(uint8_t i = 0; i<10;i++)
		{
			m = (float) (Data_ETH_old_value[i] * 0.1428 - 885.71);
			vect_val[i] = (uint32_t) m;
		}

		if(Data_ETH_old_value[10]>1000 && Data_ETH_old_value[10]<15000)
			vect_val[10] = Data_ETH_old_value[10];
		if(Data_ETH_old_value[11]>1000 && Data_ETH_old_value[11]<15000)
			vect_val[11] = Data_ETH_old_value[11];
		if(Data_ETH_old_value[12]>1000 && Data_ETH_old_value[12]<15000)
			vect_val[12] = Data_ETH_old_value[12];

	}
	break;

	case 2:
	{
//		y1 = 0;
//		x1 = 370;
//		y2 = 100;
//		x2 = 450;
//		m = ((y2 - y1)/(x2-x1));
//		m = 1.25;
//		y1 = m*x1 + n;
//		n = -(m*x1) + y1;
//		n= -100;
//		y = m*6500 + n;

		for(uint8_t i = 0; i<10;i++)
		{
			m = (float) (Data_LTC_old_value[i] * 1.25 - 462.5);
			vect_val[i] = (uint32_t) m;
		}

		if(Data_LTC_old_value[10]>100 && Data_LTC_old_value[10]<1500)
			vect_val[10] = Data_LTC_old_value[10];
		if(Data_LTC_old_value[11]>100 && Data_LTC_old_value[11]<1500)
			vect_val[11] = Data_LTC_old_value[11];
		if(Data_LTC_old_value[12]>100 && Data_LTC_old_value[12]<1500)
			vect_val[12] = Data_LTC_old_value[12];
	}
	break;

	default:
		break;

	}

	if(((max[index] < vect_val[10]) ||  (min[index] > vect_val[10])) && control)
	{
		send_alarm();
		control = 0;
	}
	else if ((max[index] > vect_val[10]) &  (min[index] < vect_val[10]))
		control = 1;



	//#endif

	return &vect_val[0];
}

