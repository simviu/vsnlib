/*
   Author: Sherman Chen
   Create Time: 2022-07-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */



#include "vsn/vsnTool.h"
#include <opencv2/ccalib/multicalib.hpp>

using namespace app;

//----
CmdVideo::CmdVideo():
    Cmd("Video commands")
{
    //---- 'frames'
    {
        string sH = "frame by frame examine and operations \n";
        sH += "   Usage: frames file=<FILE> idx=<IDX> wdir=<WDIR> [-ui]\n";
        sH += "       Notes - <IDX> can set 'all' \n";
        sH += "             - in '-ui' mode, 's' key to save frm  \n";
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
    //---- 'encode'
    {
        string sH = "encode video from images\n";
        sH += "   Usage: encode dir=<IMG_DIR> wfile=<VIDEO_FILE> \n";
        add("enc", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_enc(args); }));
    }
}

//------
bool CmdVideo::run_frames(CStrs& args)
{
    StrTbl kv;   parseKV(args, kv);
    string sf = lookup(kv, string("file"));
    string s_idx = lookup(kv, string("idx"));
    string s_wd  = lookup(kv, string("wdir"));
    if(s_wd=="") s_wd = "./";
    data_.s_wdir = s_wd;
    bool b_ui = has(kv, "-ui");
    //----
    int idx = 0;
    if(s_idx=="all") idx = -1;
    else if(!s2d(s_idx, idx))
    {
        log_e("  Invalid index:"+s_idx);
        return false;
    }
    //---
    stringstream s;
    s << "Extract video: '" << sf 
        << "', idx" << s_idx 
        << ", write to '" << s_wd <<"'..." << endl;

    //----
    auto pv = vsn::Video::open(sf);
    if(pv==nullptr) return false;
    auto& vd = *pv;
    Sp<Img> p = nullptr;
    auto& fi = data_.frm_idx;
    bool ok = true;
    while((p=vd.read())!=nullptr)
    {
        fi++;
        auto& im = *p;
        im.show(sf);

        //---- save all frm
        if(idx==-1)
        {
            ok = save_frm(im);
            if(!ok) return false;
            continue;
        }
        // save one frm
        else if(idx==fi)
        {
            ok = save_frm(im);
            break;
        }
        else if(!b_ui)
            continue;

        //--- interactive mode
        ok = video_frm_ui(im);
        if(!ok) break;
        
    }
    return ok;
}
//------
bool CmdVideo::save_frm(const Img& im)
{
    auto& fi = data_.frm_idx;
    string sf= data_.s_wdir +"/"+ 
                std::to_string(fi)+".png";
    if(!im.save(sf))
    {
        log_ef(sf);
        return false;
    }   
    log_i("saved: "+sf); 
    return true;
}

//------
bool CmdVideo::video_frm_ui(const Img& im)
{
    auto& fi = data_.frm_idx;
    while(1)
    {
        //---- keyboard handle
        int k = vsn::cv_waitkey(10);
        switch(k)
        {
            case ' ':return true; // next frm
            //--- save
            case 's' : 
                if(!save_frm(im))
                    return false; 
                break; 

            default: break;
        }
    }

    return true;
}



//------
bool CmdVideo::run_crop(CStrs& args)
{
    StrTbl kv;   parseKV(args, kv);
    string sf  = lookup(kv, string("file"));
    string sfw = lookup(kv, string("filew"));
    if(sf=="" || sfw=="")
    {
        log_e("  file or filew not provided");
        return false;
    }
        
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


//------
bool CmdVideo::run_enc(CStrs& args)
{
    StrTbl kv;   parseKV(args, kv);
    string sdir = lookup(kv, string("dir"));
    string sfw  = lookup(kv, string("filew"));
    //--- all imgs
    vector<string> sfs;
    cv::glob(sdir, sfs);
    //---- get first
    if(sfs.size()==0)
    { log_e("  No img found"); return false; }
    auto p_im0 = Img::loadFile(sfs[0]);
    assert(p_im0!=nullptr);
    Sz sz = p_im0->size();
        
    //---- creat video
    Video::Cfg vc{sz, 30};
    auto p_vd = Video::create(sfw, vc);
    if(p_vd==nullptr) 
        return false;
    for(auto& sf : sfs)
    {
        auto p = Img::loadFile(sf);
        p_vd->write(*p);

    }
    p_vd->close();
    return true;
}

