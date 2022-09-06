/*
   Author: Sherman Chen
   Create Time: 2022-07-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */



#include "vsn/vsnTool.h"

using namespace app;

//----
CmdVideo::CmdVideo():
    Cmd("Video commands")
{
    //---- 'frames'
    {
        string sH = "frame by frame examine and operations \n";
        sH += "   Usage: frames file=<FILE> \n";
        add("frames", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_frames(args); }));
    }

}

//------
bool CmdVideo::run_frames(CStrs& args)
{
    StrTbl kv;   parseKV(args, kv);
    string sf = lookup(kv, string("file"));
    auto pv = vsn::Video::open(sf);
    if(pv==nullptr) return false;
    auto& vd = *pv;
    Sp<Img> p = nullptr;
    auto& fi = data_.frm_idx;
    while((p=vd.read())!=nullptr)
    {
        fi++;
        auto& im = *p;
        im.show(sf);
        if(!video_frm_ui(im))
            break;
    }
    return true;
}
//------
bool CmdVideo::video_frm_ui(const Img& im)
{
    auto& fi = data_.frm_idx;
    //---- keyboard handle
    while(1)
    {
        int k = vsn::cv_waitkey(10);
        switch(k)
        {
            case ' ':return true; // next frm
            //--- save
            case 's' : {
                string sf="frm"+std::to_string(fi)+".jpg";
                im.save(sf);
            }; break; 

            default: break;
        }
    }

    return true;
}


