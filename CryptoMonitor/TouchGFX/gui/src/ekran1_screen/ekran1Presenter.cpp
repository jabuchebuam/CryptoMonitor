#include <gui/ekran1_screen/ekran1View.hpp>
#include <gui/ekran1_screen/ekran1Presenter.hpp>

ekran1Presenter::ekran1Presenter(ekran1View& v)
    : view(v)
{

}

void ekran1Presenter::activate()
{

}

void ekran1Presenter::deactivate()
{

}

void ekran1Presenter::GetValue()
{

}

uint32_t ekran1Presenter::GetMinVal(uint8_t index)
{
	return model->GetMinVal(index);
}

uint32_t ekran1Presenter::GetMaxVal(uint8_t index)
{
	return model->GetMaxVal(index);
}

void ekran1Presenter::SetMinVal(uint8_t index, uint32_t val)
{
	model->SetMinVal(index, val);
}

void ekran1Presenter::SetMaxVal(uint8_t index, uint32_t val)
{
	model->SetMaxVal(index, val);
}

uint32_t* ekran1Presenter::Get_Vect_Val(uint8_t index)
{
	uint32_t value;
	return model->Get_Vect_Val(index);
}

