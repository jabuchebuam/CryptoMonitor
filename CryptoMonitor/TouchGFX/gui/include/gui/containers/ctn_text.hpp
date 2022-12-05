#ifndef CTN_TEXT_HPP
#define CTN_TEXT_HPP

#include <gui_generated/containers/ctn_textBase.hpp>

class ctn_text : public ctn_textBase
{
public:
    ctn_text();
    virtual ~ctn_text() {}

    virtual void initialize();

    void set_text(uint8_t *str);

protected:
};

#endif // CTN_TEXT_HPP
