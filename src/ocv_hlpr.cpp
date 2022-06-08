#include "vsn/ocv_hlpr.h"

using namespace ocv;
using namespace vsn;
using namespace ut;
using namespace cv;

extern string str(const cv::Point3d& d, int decim)
{
    stringstream ss;
    ss.setf(ios::fixed,ios::floatfield);
    ss.precision(decim);
    ss << std::fixed << d.x <<", "<< d.y <<", " << d.z;
    return ss.str();
}
//------------
// dbg
//------------
extern bool dbg_showImgWind()
{
    static int cnt_=0;
    cnt_++;

    Mat im(300, 500, CV_8UC3, Scalar(100, 20, 20));
    string s = " OpenCV Dbg Window "+std::to_string(cnt_);
    
    cv::putText(im,s,{30,30},cv::FONT_HERSHEY_DUPLEX,
        1 ,{255,255,255}, 2, false);
    cv::imshow("dbg", im);
    char key = (char) cv::waitKey(5);
    return  (key != 27); // quit on ESC
}
