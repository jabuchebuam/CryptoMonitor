#ifndef EKRAN1PRESENTER_HPP
#define EKRAN1PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ekran1View;

class ekran1Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    ekran1Presenter(ekran1View& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    float data[10];

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~ekran1Presenter() {};

    uint32_t GetMinVal(uint8_t index);
    uint32_t GetMaxVal(uint8_t index);
    void SetMinVal(uint8_t index, uint32_t val);
    void SetMaxVal(uint8_t index, uint32_t val);

    uint32_t* Get_Vect_Val(uint8_t index);

    virtual void GetValue();

private:
    ekran1Presenter();

    ekran1View& view;
};

#endif // EKRAN1PRESENTER_HPP
