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

    //---- 'crop'
    {
        string sH = "crop video \n";
        sH += "   Usage: crop file=<FILE> \n";
        add("crop", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_crop(args); }));
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



//------
bool CmdVideo::run_crop(CStrs& args)
{
    StrTbl kv;   parseKV(args, kv);
    string sf = lookup(kv, string("file"));
        
    auto p_vd = vsn::Video::open(sf);
    if(p_vd==nullptr)
        return false;
    auto& vd = *p_vd;

    //----
    bool ok = true;
    Px px;  ok &= px.set(lookup(kv, "start"));
    Sz sz;  ok &= sz.set(lookup(kv, "sz")); 
    Sz vdsz = vd.cfg_.sz;
    if(!ok)
    {
        log_e("  Parsing arg fail");
        return false;
    }

    //---- command print
    string s = "Crop video: src sz=" + vdsz.str();
    s += ", start=" + px.str();
    s += ", sz=" + sz.str();
    log_i(s);
    
    //---- Left/Right img
    Px rcs{px.x +  sz.w   /2, px.y + sz.h/2};
    Color cr{255,0,0,255};
    Rect r(rcs, sz);
    
    //---- Open video to write
    FPath fp(sf);
    string sfw = fp.base + "_crop" + fp.ext;
    auto wvc = vd.cfg_;
    wvc.sz = sz;
    auto p_vdw = Video::create(sfw, wvc);
    if(p_vdw==nullptr)
    {
        log_ef(sfw);
        return false;
    }
    //----
    while(true)
    {
        auto p_im = vd.read();
        if(p_im==nullptr) break;
        auto p_imc = p_im->crop(r);
        if(p_imc==nullptr)
        {
            string s = "  Rect out of range, ";
            s += " Video sz:[" + vdsz.str() + "], ";
            s += "crop rect:[" + r.p0().str()+"->"+ 
                r.p1().str() +"]\n";
            log_e(s);
            return false;
        }
        //----
        p_vdw->write(*p_imc);

    }
    //---- done
    p_vdw->close();
    
    return true;
}
