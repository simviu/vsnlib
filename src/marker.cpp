/*
   Author: Sherman Chen
   Create Time: 2022-05-17
   Email: schen@simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"
#include <jsoncpp/json/json.h>

using namespace vsn;
using namespace cv;
using namespace ocv;

//---------------
string Marker::str()const
{
    // in json stream 
    stringstream ss;
    ss << "id:" << id << ", " << endl;
    ss << "corners:[" ;
    for(int i=0;i<4;i++)
    {
        if(i!=0) ss << ", ";
        vec2 q = ps[i];
        ss << '"' << q.x() << "," << q.y() << '"'; 
    }
    ss << "]" << endl;
    //----
    ss << "pose:{"  << pose.str() <<"}";
    return ss.str();

}
     
     
//-----
string Marker::Cfg::str()const
{ 

    Json::Value jgs;
    for(auto& g : grps_) 
    {
        //----
        Json::Value jids;
        for(auto& i :g.ids)
            jids.append(i);
        //--- 
        Json::Value jg;
        jg["ids"] = jids;  
        jg["w"] = g.w; 
        jgs.append(jg);
    }
    //----
    Json::Value jd;
    jd["groups"] = jgs;
    jd["aruco_dict"] = sDict_;
    jd["aruco_dict_id"] = dict_id_;
    //----
    stringstream s; 
    s << jd;
    return s.str();
}

//---------------
bool Marker::Cfg::load(CStr& sf)
{

    ifstream ifs(sf);
    if(!ifs)
    {
        log_ef(sf);
        return false;
    }
    //----
    try{

        Json::Reader rdr;
        Json::Value jd;
        rdr.parse(ifs, jd);
        auto& jm = jd["marker_cfg"];
        auto& jgs = jm["groups"];
        sDict_ = jm["aruco_dict"].asString();
        dict_id_ = jm["aruco_dict_id"].asInt();
        for(auto& jg : jgs)
        {
            Grp g;
            g.w = jg["w"].asDouble();
            //----
            auto jids = jg["ids"];
            for(auto& ji : jids)
                g.ids.insert(ji.asInt());
            //----
            grps_.push_back(g);
        } 
        //
        //cout << " name " << obj["name"].asString() << endl;
    }
    catch(exception& e)
    {
        log_e("exception caught:"+string(e.what()));
        return false;
    }
    //---- dbg
    string s = this->str();
    log_d(s);
    return true;
}

//---------------
bool Marker::detect(const Img& im,
                    vector<Marker>& ms,
                    int dict_id)
{
    ocv::ImgCv imc(im);
    //default: 0,  cv::aruco::DICT_5X5_250    
    //---- local static data
    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(dict_id);
    vector<int> ids;
    vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(imc.im_, dict, corners, ids);
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
bool Marker::detect(const Img& im, 
                    const Cfg& cfg,
                    const CamCfg& camc,
                    vector<Marker>& ms)
{
    bool ok = true;
    int dict_id = cfg.dict_id_;
    for(auto& g : cfg.grps_)
    {
        vector<Marker> gms;
        detect(im, gms, dict_id);
        //---- pose estimate
        for(auto& m : gms)
        {
            auto& ids = g.ids;
            auto it = ids.find(m.id);
            if(it==ids.end())continue;

            ok &= m.pose_est(camc, g.w);
            ms.push_back(m);
        }
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