#include <gui/ekran1_screen/ekran1View.hpp>
#include "stdio.h"
#include <touchgfx/Color.hpp>

#include "string.h"

ekran1View::ekran1View()
{

}

void ekran1View::setupScreen()
{
	ekran1ViewBase::setupScreen();
	current_index = 0xFF;
}

void ekran1View::tearDownScreen()
{
	ekran1ViewBase::tearDownScreen();
}

void ekran1View::LED_FONK()
{

}

void ekran1View::LeftCrypto()
{
	uint8_t aux;
	aux = scrollWheel1.getSelectedItem();
	aux--;
	scrollWheel1.animateToItem(aux,0);
	scrollWheel1.invalidate();

}

void ekran1View::RightCrypto()
{
	uint8_t aux;
	aux = scrollWheel1.getSelectedItem();
	aux++;
	scrollWheel1.animateToItem(aux,0);
	scrollWheel1.invalidate();
}

void ekran1View::DownValueMin()
{
	min = presenter->GetMinVal(current_index);
	min--;
	presenter->SetMinVal(current_index, min);
}

void ekran1View::UpValueMin()
{
	min = presenter->GetMinVal(current_index);
	min++;
	presenter->SetMinVal(current_index, min);
}

void ekran1View::DownValueMax()
{
	max = presenter->GetMaxVal(current_index);
	max--;
	presenter->SetMaxVal(current_index, max);

}

void ekran1View::UpValueMax()
{
	max = presenter->GetMaxVal(current_index);
	max++;
	presenter->SetMaxVal(current_index, max);
}

void ekran1View::GetValue()
{

}

void ekran1View:: handleTickEvent()
{
	uint8_t str[15];

	scrollWheel1.itemChanged(current_index);
	memcpy(&graph_val[0], presenter->Get_Vect_Val(current_index), 13*4);

	min = presenter->GetMinVal(current_index);
	sprintf((char*)&str[0],"%d", min);
	Unicode::fromUTF8(str, ta_minBuffer, TA_MIN_SIZE);
	if(graph_val[10] < min)
		ta_min.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		ta_min.setColor(Color::getColorFromRGB(0xFF, 0xFF, 0xFF));
	ta_min.invalidate();

	max = presenter->GetMaxVal(current_index);
	sprintf((char*)&str[0],"%d", max);
	Unicode::fromUTF8(str, ta_maxBuffer, TA_MIN_SIZE);
	if(graph_val[10] > max)
		ta_max.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		ta_max.setColor(Color::getColorFromRGB(0xFF, 0xFF, 0xFF));
	ta_max.invalidate();

	if(current_index != scrollWheel1.getSelectedItem())
	{
		current_index = scrollWheel1.getSelectedItem();
		min = presenter->GetMinVal(current_index);
		if(!min)
			min = graph_val[12];
		sprintf((char*)&str[0],"%d", min);
		Unicode::fromUTF8(str, ta_minBuffer, TA_MIN_SIZE);
		ta_min.invalidate();

		max = presenter->GetMaxVal(current_index);
		if(!max)
			max = graph_val[11];
		sprintf((char*)&str[0],"%d", max);
		Unicode::fromUTF8(str, ta_maxBuffer, TA_MIN_SIZE);
		ta_max.invalidate();

	}
	else
	{
		if(max!=sw_max.getSelectedItem())
		{
			max=sw_max.getSelectedItem();
		}
		if(min!=sw_min.getSelectedItem())
		{
			min=sw_min.getSelectedItem();
		}
	}

	set_bars();

}

void ekran1View::set_bars(void)
{

	uint8_t str[30];

	bp_1.setValue(graph_val[0]);

	if(graph_val[1]<graph_val[0])
		bp_1_1.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_1.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_1.setValue(graph_val[1]);

	if(graph_val[2]<graph_val[1])
		bp_1_2.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_2.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_2.setValue(graph_val[2]);

	if(graph_val[3]< graph_val[2])
		bp_1_3.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_3.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_3.setValue(graph_val[3]);

	if(graph_val[4]< graph_val[3])
		bp_1_4.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_4.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_4.setValue(graph_val[4]);

	if(graph_val[5]< graph_val[4])
		bp_1_5.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_5.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_5.setValue(graph_val[5]);

	if(graph_val[6]< graph_val[5])
		bp_1_6.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_6.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_6.setValue(graph_val[6]);

	if(graph_val[7]< graph_val[6])
		bp_1_7.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_7.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_7.setValue(graph_val[7]);

	if(graph_val[8]< graph_val[7])
		bp_1_8.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_8.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_8.setValue(graph_val[8]);

	if(graph_val[9]< graph_val[8])
		bp_1_9.setColor(Color::getColorFromRGB(0x00, 0xFF, 0x00));
	else
		bp_1_9.setColor(Color::getColorFromRGB(0x00, 0x00, 0xFF));
	bp_1_9.setValue(graph_val[9]);

	sprintf((char*)&str[0],"%d", graph_val[11]);
	Unicode::fromUTF8(str, textArea1Buffer, 10);
	sprintf((char*)&str[0],"%d", graph_val[12]);
	Unicode::fromUTF8(str, textArea1_1Buffer, 10);
	sprintf((char*)&str[0],"%d", graph_val[10]);
	Unicode::fromUTF8(str, textArea1_1_1Buffer, 10);

	textArea1.invalidate();
	textArea1_1.invalidate();
	textArea1_1_1.invalidate();

	box1.invalidate();
	bp_1.invalidate();
	bp_1_1.invalidate();
	bp_1_2.invalidate();
	bp_1_3.invalidate();
	bp_1_4.invalidate();
	bp_1_5.invalidate();
	bp_1_6.invalidate();
	bp_1_7.invalidate();
	bp_1_8.invalidate();
	bp_1_9.invalidate();

}

void ekran1View::scrollWheel1UpdateItem(ctn_text& item, int16_t itemIndex)
{
	uint8_t str[30];

	switch(itemIndex)
	{
	case 0: sprintf((char*)&str[0],"BTC"); break;
	case 1: sprintf((char*)&str[0],"ETH"); break;
	case 2: sprintf((char*)&str[0],"LTC"); break;
	case 3: sprintf((char*)&str[0],"BCH"); break;
	case 4: sprintf((char*)&str[0],"SOL"); break;
	case 5: sprintf((char*)&str[0],"ADA"); break;
	case 6: sprintf((char*)&str[0],"LINK"); break;
	case 7: sprintf((char*)&str[0],"GNO"); break;
	default : sprintf((char*)&str[0],"BTC"); break;

	}

	item.set_text(&str[0]);
}

void ekran1View::sw_minUpdateItem(ctn_text& item, int16_t itemIndex)
{
	uint8_t str[30];

	sprintf((char*)&str[0],"%d", itemIndex);

	item.set_text(&str[0]);
}

void ekran1View::sw_maxUpdateItem(ctn_text& item, int16_t itemIndex)
{
	uint8_t str[30];

	sprintf((char*)&str[0],"%d", itemIndex);

	item.set_text(&str[0]);
}
