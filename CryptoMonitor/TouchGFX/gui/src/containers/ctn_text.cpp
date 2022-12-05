#include <gui/containers/ctn_text.hpp>

ctn_text::ctn_text()
{

}

void ctn_text::initialize()
{
	ctn_textBase::initialize();
}

void ctn_text::set_text(uint8_t *str)
{
	Unicode::fromUTF8(str, ta_textBuffer, TA_TEXT_SIZE);
	ta_text.invalidate();
}
