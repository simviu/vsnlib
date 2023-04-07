/*
   Author: Sherman Chen
   Create Time: 2022-07-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */



#include "vsn/vsnTool.h"
#include <opencv2/ccalib/multicalib.hpp>
#include <filesystem>

using namespace app;

namespace {
    
}

//----
CmdVideo::CmdVideo():
    Cmd("Video commands")
{
    //---- 'frames'
    {
        string sH = "frame by frame examine and operations \n";
        sH += "   Usage: frames file=<FILE> idx=<IDX> wdir=<WDIR> [-ui] [-stereo]\n";
        sH += "       Notes : in '-ui' mode, 's' key to save frm  \n";
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
    //StrTbl kv;   parseKV(args, kv);
    KeyVals kvs(args);
    string sf; 
    if(!kvs.get("file", sf)) return false;
    string s_wd  = kvs["wdir"];
    string s_idx = kvs.query("idx");
    data_.is_stereo = kvs.has("-stereo");
    //----
    if(s_wd=="") s_wd = "./";
    data_.s_wdir = s_wd;
    if(!sys::mkdir(s_wd))return false;
    //---
    bool b_ui = kvs.has("-ui");
    //----
    int idx = -1;
    if ( (s_idx!="")&&
         (!s2d(s_idx, idx)) )
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
    //---- stereo
    if(data_.is_stereo)
         return save_frm_stereo(im);
    
    //----
    auto& fi = data_.frm_idx;
    string sf = data_.s_wdir + "/" + std::to_string(fi)+".png";
    bool ok = im.save(sf);
    
    if(ok)
         log_i("saved: "+sf); 
    else log_ef(sf);
        
    return ok;
}
//----
bool CmdVideo::save_frm_stereo(const Img& im)
{
    // crop split image L/R
    Sz sz = im.size();
    sz.w *= 0.5;
    float xcs[2]{0.5, 1.5}; // center x of Rect
    string sds[2]{"/L/", "/R/"};

    //--- check/create dir
    string swds[2];
    for(int i=0;i<2;i++)
    {
        string swd = data_.s_wdir + sds[i] ;
        if(!sys::mkdir(swd)) return false;
        swds[i] = swd;
    }

    //--- write files
    auto& fi = data_.frm_idx;
    bool ok = true;
    for(int i=0;i<2;i++)
    {
        int xc = sz.w*xcs[i];
        Px pc{xc, (int)(sz.h*0.5)};
        auto p = im.crop({pc, sz});
        string sf = swds[i] + std::to_string(fi)+".png";
        ok &= p->save(sf);
    }
    return ok;
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
    if(has(kv, "-stereo"))
        return run_crop_stereo(args);

    //---- normal crop
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
    Sz vdsz = vd.cfg_.sz;
    auto p_im0 = vd.read(); // read 1st frm for some calculation

    //--------
    Px px;  Sz sz = vdsz;
    bool ok = true;

    ok &= px.set(lookup(kv, "start"));
    Sz szi; ok &= szi.set(lookup(kv, "sz")); 

    //----
    if(!ok)
    {
        log_e("  Parsing arg fail");
        return false;
    }
    if(szi.w!=-1) sz.w = szi.w;
    if(szi.h!=-1) sz.h = szi.h;

    //----- find auto y-start
    if(px.y==-1)
    {
        auto& im = *p_im0;
        //---- top boarder
        for(int y=0;y<vdsz.h;y++)
        {
            Color c; 
            bool b_blk = true;
            for(int x=0;x<vdsz.w;x++)
            {
                if(!im.get(Px(x, y), c)) continue;
                if(c.isBlack()) continue;
                b_blk = false; break;
            }
            if(b_blk)continue;

            //---- found none black
            px.y = y; 
            stringstream s;
            s << "  Found auto y start:" << y;
            log_i(s.str()); break;
        }
        //----
        if(px.y==-1)
        {
            log_e("  Not found auto y");
            return false;
        }

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
    sys::FPath fp(sf);

    auto wvc = vd.cfg_;
    wvc.sz = sz;
    auto p_vdw = Video::create(sfw, wvc);
    if(p_vdw==nullptr)
    {
        log_ef(sfw);
        return false;
    }

    //----
    int i=0;
    while(true)
    {
        auto p_im = p_im0;
        if(i>0) p_im = vd.read();
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
        i++;
    }
    //---- done
    p_vdw->close();

    {
        stringstream s;
        s << "write to video:'"+sfw +"' ok, ";
        s << "frms:" << i ;
        log_i(s.str());
    }
    return true;
}


//------
bool CmdVideo::run_crop_stereo(CStrs& args)
{
    StrTbl kv; parseKV(args, kv);
    //---- normal crop
    string sf  = lookup(kv, string("file"));
    if(sf=="")
    {
        log_e("  file not provided");
        return false;
    }
    //----
    sys::FPath fp(sf);
    string swd = lookup(kv, string("wdir"));
    if(swd=="") 
         swd = fp.path + "/";
    else swd += "/";

    //---- open video        
    auto p_vd = vsn::Video::open(sf);
    if(p_vd==nullptr)
        return false;
    auto& vd = *p_vd;
    Sz vdsz = vd.cfg_.sz;
    Sz sz = vdsz;
    sz.w /= 2;
   
    bool ok = true;
    int xs=0,ys=0; 
    string s_xs = lookup(kv, "xs");
    string s_ys = lookup(kv, "ys");
    string s_sz = lookup(kv, "sz");
    if(s_xs!="") ok &= s2d(s_xs, xs);
    if(s_ys!="") ok &= s2d(s_ys, ys);
    if(s_sz!="") ok &= sz.set(s_sz); 

    if(!ok)
    {
        log_e("  Parsing arg fail");
        return false;
    }
    //-----
    {
        stringstream s;
        s << "Crop video: src sz="  << vdsz.str();
        s << ", ys=" << ys;
        log_i(s.str());
    }
    
    //---- auto calc y offset if set -1
    auto p_im0 = vd.read();
    if(ys==-1)
    {
        stringstream s;
        auto& im = *p_im0;
        for(int y =0; y<sz.h; y++)
        {
            Color c;
            if(!im.get(Px(xs+10, y), c)) break;
            if(c.isBlack())continue;
            ys = y; break;
            s << "  Found auto ys = " << y;
            log_i(s.str());
        }
        if(ys==-1)
        {
            log_e("   Auto y offset failed, set ys manally");
            return false;
        }

    }

    //---- Left/Right img
    Px rcsL{xs +  sz.w/2, ys + sz.h/2};
    //---- assume Right rect start in middle
    Px rcsR{vdsz.w/2 + sz.w/2, ys + sz.h/2};
    Rect rL(rcsL, sz);
    Rect rR(rcsR, sz);

    Color cr{255,0,0,255};
    
    //---- Open video to write

    auto wvc = vd.cfg_;
    wvc.sz = sz;
    auto p_vL = Video::create(swd + "L.mkv", wvc);
    auto p_vR = Video::create(swd + "R.mkv", wvc);
    if(p_vL==nullptr || p_vR==nullptr)
    {
        log_e("Fail to create videos in dir:"+swd);
        return false;
    }
    //----
    int i=0;
    while(true)
    {
        auto p_im = p_im0;
        if(i>0) p_im = vd.read();
        if(p_im==nullptr) break;
        auto p_imL = p_im->crop(rL);
        auto p_imR = p_im->crop(rR);
        if(p_imL==nullptr || p_imR==nullptr)
        {
            string s = "  Rect out of range, ";
            s += " Video sz:[" + vdsz.str() + "], ";
            s += "    crop rectL:[" + rL.p0().str()+"->"+ 
                rL.p1().str() +"]\n";
            s += "    crop rectR:[" + rR.p0().str()+"->"+ 
                rR.p1().str() +"]\n";
            log_e(s);
            return false;
        }
        //----
        p_vL->write(*p_imL);
        p_vR->write(*p_imR);
        i++;
    }
    
    //---- done
    p_vL->close();
    p_vR->close();
    log_i("  Total frms:"+to_string(i));
    log_i("  Crop into L/R.mkv in dir:"+swd);
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

