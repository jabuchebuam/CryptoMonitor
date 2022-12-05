#ifndef EKRAN1VIEW_HPP
#define EKRAN1VIEW_HPP

#include <gui_generated/ekran1_screen/ekran1ViewBase.hpp>
#include <gui/ekran1_screen/ekran1Presenter.hpp>

class ekran1View : public ekran1ViewBase
{
public:
    ekran1View();
    virtual ~ekran1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    void handleTickEvent();
    virtual void LED_FONK();

    virtual void LeftCrypto();
    virtual void RightCrypto();
    virtual void DownValueMin();
    virtual void UpValueMin();
    virtual void DownValueMax();
    virtual void UpValueMax();

	virtual void GetValue();

	float data[10];

protected:
	uint8_t current_index;
	uint32_t min, max;
	uint8_t item_index;

	uint32_t graph_val[13];
	uint32_t temp[13];

	void set_bars(void);

	void scrollWheel1UpdateItem(ctn_text& item, int16_t itemIndex);
	void sw_minUpdateItem(ctn_text& item, int16_t itemIndex);
	void sw_maxUpdateItem(ctn_text& item, int16_t itemIndex);
};

#endif // EKRAN1VIEW_HPP
