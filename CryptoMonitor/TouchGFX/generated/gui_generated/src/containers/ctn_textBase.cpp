/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/containers/ctn_textBase.hpp>
#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Color.hpp>

ctn_textBase::ctn_textBase()
{
    setWidth(139);
    setHeight(39);
    ta_text.setPosition(0, 0, 139, 39);
    ta_text.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    ta_text.setLinespacing(0);
    Unicode::snprintf(ta_textBuffer, TA_TEXT_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_L6HX).getText());
    ta_text.setWildcard(ta_textBuffer);
    ta_text.setTypedText(touchgfx::TypedText(T___SINGLEUSE_K8RX));

    box1.setPosition(11, 96, 193, 94);
    box1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));

    add(ta_text);
    add(box1);
}

ctn_textBase::~ctn_textBase()
{

}

void ctn_textBase::initialize()
{

}
