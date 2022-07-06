/*
   Author: Sherman Chen
   Create Time: 2022-05-17
   Email: schen@simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"

using namespace vsn;
using namespace cv;
using namespace ocv;

//---------------
string Marker::str()const
{
    stringstream ss;
    ss << "id:" << id << ", ";
    ss << "ps:";
    for(auto& p : ps)
        ss << p << ", ";
    ss << "pose:{"  << pose.str() <<"}";
    return ss.str();

}


//---------------
bool Marker::detect(const Img& im,
                    vector<Marker>& ms)
{
    ocv::ImgCv imc(im);
    
    //---- local static data
    struct Data{
        cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    };
    static Data d_;
    vector<int> ids;
    vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(imc.im_, d_.dict, corners, ids);
    int i=0;
    for(auto& id : ids)
    {
        Marker m;
        m.id = id;
        for(int j=0;j<4;j++)
        {
            cv::Point2f c = corners[i][j];
            vec2 p;p<<c.x, c.y;
            m.ps[j] = p;
  
        }
        
        
        //----
        ms.push_back(m);
        i++;
    }
    return true;
}
//-----------
bool Marker::pose_est(const CamCfg& cc, double w)
{

    cv::Mat Kc,Dc;
    eigen2cv(cc.K, Kc);
    vec5 Dv = cc.D.V();
    eigen2cv(Dv, Dc);
    //---- pose estimation for this marker
    vector<Vec3d> rs, ts;
    vector<Point2f> cs;

    for(int i=0;i<4;i++)
        cs.push_back(Point2f(ps[i].x(), ps[i].y()));
    vector<vector<cv::Point2f>> corners{cs};

    aruco::estimatePoseSingleMarkers(corners, 
        w, Kc, Dc, rs, ts);
        
    Mat Rc; cv::Rodrigues(rs[0], Rc);
    mat3 R; cv::cv2eigen(Rc, R);
    vec3 t; cv::cv2eigen(ts[0], t);
    pose.q = quat(R);
    pose.t = t;
    return true;
}

//---------------
bool Marker::fit_plane(
    const vector<Marker>& ms, Pose& p)
{
    // average quat
    int i=0;

    for(const auto& m : ms)
    {

        if(i==0)
        {
            p.q = m.pose.q;
            i++;
            continue;
        }
        //---- average quat by slerp()
        double t = 1.0/(i+1.0); 
        p.q = p.q.slerp(t, m.pose.q);
        i++;
    }
    //---- average t
    int N = ms.size();
    vec3 t; t << 0,0,0;
    for(auto& m : ms)
        t += m.pose.t;
    p.t = t*(1.0/N);
    return true;
}