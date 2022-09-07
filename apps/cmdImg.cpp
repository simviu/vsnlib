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
    void mouse_callbk(int event, int x, int y, int flags, void* userdata)
    { 
        //function to track mouse movement and click//
        using namespace cv;
        if (event == EVENT_LBUTTONDOWN){ //when left button clicked//
            cout << "Left click has been made, Position:(" << x << "," << y << ")" << endl;
        } else if (event == EVENT_RBUTTONDOWN){ //when right button clicked//
            cout << "Rightclick has been made, Position:(" << x << "," << y << ")" << endl;
        } else if (event == EVENT_MBUTTONDOWN){ //when middle button clicked//
            cout << "Middleclick has been made, Position:(" << x << "," << y << ")" << endl;
        } else if (event == EVENT_MOUSEMOVE){ //when mouse pointer moves//
            cout << "Current mouse position:(" << x << "," << y << ")" << endl;
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
    StrTbl kv;   parseKV(args, kv);
    string sf = lookup(kv, string("file"));
    auto p_im = vsn::Img::create();
    auto& imi = *p_im;
    if(!imi.load(sf))
        return false;
    cv::Mat im = ImgCv(imi).raw();
    cv::namedWindow(sf);//declaring window to show image//
    cv::setMouseCallback(sf, mouse_callbk, NULL);//Mouse callback function on define window//
    cv::imshow(sf, im);//showing image on the window//
    cv::waitKey(0);//wait for keystroke//
    return true;
}
