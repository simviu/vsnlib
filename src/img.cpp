#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"
using namespace vsn;
using namespace ut;


//----factory implementation use ImgCv;
Sp<Img> Img::create()
{
    return make_shared<ocv::ImgCv>();
}

