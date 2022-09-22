/*
   Author: Sherman Chen
   Create Time: 2022-09-07
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */



#include "vsn/vsnTool.h"
#include "vsn/vsnLibCv.h"

using namespace app;

//----
namespace{
    //-------
    // picker
    //-------
    namespace picker{
        struct UserDt{
            Sp<Img> p_im = nullptr;
            Sp<Img> p_imh = nullptr; // hsv
            string sWin;

        };
        //-----
        void mouse_callbk(int event, int x, int y, int flags, void* userdata)
        { 
            using namespace cv;
            auto& ud = *reinterpret_cast<UserDt*>(userdata);
            auto p_imo = ud.p_im->copy();
            auto& im  = *ud.p_im;
            auto& imh = *ud.p_imh;
            auto& imo = *p_imo;

            Px px(x,y);
            Color co{0,0,255,255};
            if (event == EVENT_MOUSEMOVE)
            {
                //cout << "Current mouse position:(" << x << "," << y << ")" << endl;
                Color c; imo.get(px, c);
                HSV h;  imh.get(px, h);
                stringstream s1,s2,s3; 
                s1 << "[" << x << "," << y << "]";
                s2 << "rgb(" << c.str() << ")";
                s3 << "hsv(" << h.str() << ")";

                imo.draw(s1.str(), px, co);
                imo.draw(s2.str(), px +Px(0,40), co);
                imo.draw(s3.str(), px +Px(0,80), co);
            }
            imo.show(ud.sWin);
            //------- example
            //function to track mouse movement and click//
            /* dbg
            if (event == EVENT_LBUTTONDOWN){ //when left button clicked//
                cout << "Left click has been made, Position:(" << x << "," << y << ")" << endl;
            } else if (event == EVENT_RBUTTONDOWN){ //when right button clicked//
                cout << "Rightclick has been made, Position:(" << x << "," << y << ")" << endl;
            } else if (event == EVENT_MBUTTONDOWN){ //when middle button clicked//
                cout << "Middleclick has been made, Position:(" << x << "," << y << ")" << endl;
            } else if (event == EVENT_MOUSEMOVE){ //when mouse pointer moves//
                cout << "Current mouse position:(" << x << "," << y << ")" << endl;
            }*/
        }
    }
}
//----
CmdImg::CmdImg():
    Cmd("Img commands")
{
    //---- 'picker'
    {
        string sH = "color picker UI \n";
        sH += "   Usage: picker file=<IMG> \n";
        add("picker", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_picker(args); }));
    }
    //---- 'crop'
    {
        string sH = "Image cropping \n";
        sH += "   Usage: crop file=<IMG> start=x,y sz=w,h [-stereo|...]\n";
        add("crop", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_crop(args); }));
    }

}

//------
bool CmdImg::run_picker(CStrs& args)
{
    using namespace picker;
    StrTbl kv;   parseKV(args, kv);
    string sf = lookup(kv, string("file"));
    auto p_im = vsn::Img::create();
    if(!p_im->load(sf))
        return false;
    
    //----
    string sWin = sf;
    UserDt ud; 
    ud.p_im = p_im;
    ud.p_imh = p_im->copy(); 
    ud.p_imh->toHsv();
    ud.sWin = sWin;

    cv::namedWindow(sWin);//declaring window to show image//
    cv::setMouseCallback(sWin, picker::mouse_callbk, &ud);//Mouse callback function on define window//
    //cv::imshow(sf, im);//showing image on the window//
    cv::waitKey(0);//wait for keystroke//
    return true;
}

//------
bool CmdImg::run_crop(CStrs& args)
{
    StrTbl kv; parseKV(args, kv);
    string sf = lookup(kv, string("file"));
        
    auto p_im = vsn::Img::loadFile(sf);
    if(!p_im->load(sf))
        return false;
    auto& im = *p_im;
    auto p_imd = im.copy(); // dbg output
    //----
    bool ok = true;
    Px px;  ok &= px.set(lookup(kv, "start"));
    Sz sz;  ok &= sz.set(lookup(kv, "sz")); 
    Sz imsz = im.size();
    if(!ok)
    {
        log_e("  Parsing arg fail");
        return false;
    }
    bool b_stereo = has(kv, "-stereo");
    //---- command print
    string s = "Crop img: imgSz=" + imsz.str();
    s += ", start=" + px.str();
    s += ", sz=" + sz.str();
    if(b_stereo) s+= ", -stereo"; 
    log_i(s);
    
    //---- Left/Right img
    Px rcs[]{{px.x +  sz.w   /2, px.y + sz.h/2},
             {px.x + (sz.w*3)/2, px.y + sz.h/2}};
    int N = b_stereo? 2:1;
    Color cr[]{{255,0,0,255}, {0,255,0,255}};
    FPath fp(sf);

    // For stereo, crop 2 imgs, otherwise 1 img.    
    for(int i=0;i<N;i++)
    {
        Rect r(rcs[i], sz);
        auto p_imc = im.crop(r);
        string sLR = (i==0)?"_cL":"_cR";
        string sfcw = fp.base + sLR + fp.ext;
        if(p_imc==nullptr)
        {
            string s = "  Rect out of range, ";
            s += " Img sz:[" + imsz.str() + "], ";
            s += "crop rect:[" + r.p0().str()+"->"+ 
                r.p1().str() +"]\n";
            log_e(s);
            continue;
        }
        //---- save image
        p_imc->save(sfcw);
        p_imd->draw(r, cr[i]); // dbg display
        
    }
    //---- save dbg img
    p_imd->save(fp.base + "_dbg" + fp.ext);
    return true;
}
