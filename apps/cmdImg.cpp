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
