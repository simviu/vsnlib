/*
   Author: Sherman Chen
   Create Time: 2022-05-17
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/vsnLibCv.h"
#include "json/json.h"

using namespace vsn;
using namespace cv;
using namespace ocv;

//---------------
namespace{
    using MarkerPE=Marker::PoseEstimator;
    using Board=MarkerPE::Board;
    //----
    using DictPtr = cv::Ptr<cv::aruco::Dictionary>;
    struct DictionTbl{
        DictPtr findCreate(int id)
        {
           if(tbl.find(id)!=tbl.end())
              return tbl[id];
           auto p = cv::aruco::getPredefinedDictionary(id);
           tbl[id] = p;
           return p;
        }
    protected:
        map<int, DictPtr> tbl;
    };
    DictionTbl dictTbl_;
    //---- Euler to quat for ArUco marker
    // z-up, x-right, y-forward 
    quat to_q(const Euler& e)
    {
        vec3 nx,ny,nz;
        nx << 1,0,0; ny << 0,1,0; nz << 0,0,1;
        mat3 mx = rotmat(nx, toRad(e.rx));
        mat3 my = rotmat(ny, toRad(e.ry));
        mat3 mz = rotmat(nz, toRad(e.rz));
        quat q(mz * my * mx);
        return q;
    }
    //-----
    struct CvDetd{
        int dict_id = -1;
        vector<int> ids;
        vector<std::vector<cv::Point2f>> corners;
    };
    //----
    bool cv_det(const Img& im, int dict_id, CvDetd& detd)
    {
        auto pDict = dictTbl_.findCreate(dict_id);
        assert(pDict!=nullptr);
        cv::Mat imc = ImgCv(im).raw();
        stringstream s; 
        cv::aruco::detectMarkers(imc, pDict, detd.corners, detd.ids);
        detd.dict_id = dict_id;
        return true;
    }
    //--- fill result of marker det
    void fill(const CvDetd& detd, vector<Marker>& ms)
    {

        int i=0;
        for(auto& id : detd.ids)
        {
            Marker m;
            m.id = id;
            m.dict_id = detd.dict_id;
            for(int j=0;j<4;j++)
            {
                cv::Point2f c = detd.corners[i][j];
                vec2 p;p<<c.x, c.y;
                m.ps[j] = p;
            }
            
            
            //----
            ms.push_back(m);
            i++;
        }
    }
    //---------
    // board
    //---------
    using BoardCfg = Marker::PoseEstimator::Board::Cfg;
    struct BrdCfgImp : public BoardCfg
    {
        Ptr<aruco::Board> pBrd = nullptr;
        //---- load json board cfg
        bool load(const Json::Value& j)
        {
            bool ok = true;
            sName = j["name"].asString();
            string st = j["type"].asString();
            //---- normal boards
            if(st=="flat")
            {
                auto& jms = j["markers"];
                for(auto& jm : jms)
                {
                    Board::Cfg::Mark m;
                    m.id = jm["id"].asInt();
                    m.w = jm["w"].asDouble();
                    ok &= s2v(jm["xy"].asString(), m.xy);
                    marks.push_back(m);
                }
            }
            //----- banner
            else if(st=="banner")
            {
                double L = j["total_len"].asDouble();
                int ids = j["id_start"].asInt();
                double mo = j["marker_occupy"].asDouble();
                int Nm = j["num"].asInt();
                assert(Nm!=0);
                //----
                double wg = L/Nm;
                double wm = wg * mo;
                double b = (wg - wm)*0.5;
                for(int j=0;j<Nm;j++)
                {
                    Board::Cfg::Mark m;
                    m.id = ids +j;
                    m.w = wm;
                    double x = (j+0.5)*wg;
                    double y = wg/2;
                    m.xy << x, y;
                    marks.push_back(m);

                }
            }
            else{
                log_e("  not found 'type' for banner '"+ sName +"'");
                return false;
            }
            //--- check if there is transform
            if(j.isMember("T"))
            {
                Pose Twb;
                auto& jt = j["T"];

                ok &= s2v(jt["xyz"].asString(), T.t);  
                Euler e; 
                ok &= e.set(jt["euler"].asString());
                T.q = to_q(e);
            }
            //-----
            return ok;
        }
        //---- init
        virtual void init(int dict_id)override
        {
                //---- create board
            vector<vector<Point3f>> allPnts;
            vector<int> ids;
            for(auto& m : marks)
            {
                double d = m.w*0.5;
                vec3 c; c << m.xy, 0;
                //--- 4 corner points start from
                // lef/top, clock wise, on x,y plane.
                // ( x righy, y up)
                vec3 vs[4]; 
                vs[0] << -d,  d, 0; vs[1] <<  d,  d, 0;
                vs[3] << -d, -d, 0; vs[2] <<  d, -d, 0;
                vector<Point3f> ps;
                for(auto vi : vs) 
                {
                    vec3 v = vi + c;
                    ps.push_back(ocv::toCv(v));
                    box.upd(v);
                }
                ids.push_back(m.id);
                allPnts.push_back(ps);
            }
            box.upd(zerov3());// original always included
            auto pDict = dictTbl_.findCreate(dict_id);
            pBrd = aruco::Board::create(allPnts, pDict, ids);
        }
        //------
        bool det(const CvDetd& detd, const CamCfg& camc,
                Pose& pose)
        {
            assert(pBrd!=nullptr);
            cv::Mat r,t;
            cv::Mat K,D ;
            cv::eigen2cv(camc.K, K);
            cv::eigen2cv(camc.D, D);
            int valid = cv::aruco::estimatePoseBoard(detd.corners, detd.ids, pBrd, K, D, r, t);
            if(valid==0) return false;
            cv::Mat R; Rodrigues(r, R);
            mat3 Re; cv::cv2eigen(R, Re);
            //----
            Pose Tcb;
            Tcb.q = quat(Re);

            //Mat Ri;transpose(R, Ri);
            //Mat ti = - Ri * t;
            //----- Unkown hack:
            Point3f v(t);
            Tcb.t << v.x, v.y, v.z; 
            //--- further transform
            // ( pose is Tcw)
            Pose Tbo = T.inv();
            pose = Tcb * Tbo;
            return true;
        }
    };

    
}
// factory
Sp<BoardCfg> BoardCfg::create()
{ return mkSp<BrdCfgImp>();}

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
string Marker::PoseEstimator::MCfg::str()const
{ 
    // groups
    /*
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
    */
    //---- boards
    Json::Value jbrds;
    for(auto p : boards_) 
    {
        //----
        Json::Value jms;
        auto& brd = *p;
        for(auto& m :brd.marks)
        {
            Json::Value jm;
            jm["id"] = m.id;
            jm["w"] = m.w;
            jm["xy"] = egn::str(m.xy);
            jms.append(jm);
        }        
        //--- 
        Json::Value j;
        j["name"] = brd.sName;  
        j["markers"] = jms;
        jbrds.append(j);
    }
    //----
    Json::Value jd;
//    jd["groups"] = jgs;
    jd["boards"] = jbrds;
    jd["aruco_dict"] = sDict_;
    jd["aruco_dict_id"] = dict_id_;
    //----
    stringstream s; 
    s << jd;
    return s.str();
}

//---------------
bool Marker::PoseEstimator::MCfg::load(CStr& sf)
{
    log_i("Loading marker cfg:'"+sf+"'");
    bool ok = true;
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
        //----
        auto& jr = jm["run"];
        en_imo = jr["en_imo"].asBool();

        //----
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
        //---- load boards
        auto& jbrds = jm["boards"];
        for(auto& jbrd : jbrds)
        {
            auto pBc = Board::Cfg::create();
            auto& bc = static_cast<BrdCfgImp&>(*pBc);            
            if(!bc.load(jbrd)) 
                continue;

            pBc->init(dict_id_);
            boards_.push_back(pBc);
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
    
    CvDetd detd;
    cv_det(im, dict_id, detd);
    fill(detd, ms);
    return true;
}
//-----------
bool Marker::PoseEstimator::onImg(const Img& im)
{
 //   log_d(" PoseEstimator::onImg()...");
    result_ = Result();
    auto& ms = result_.ms;
    ms.clear();
    
    bool ok = true;
    auto& mc = cfg_.mcfg;
    auto& camc = cfg_.camc;
    int dict_id = mc.dict_id_;
    
    //---- detect markers
    CvDetd detd;
    cv_det(im, dict_id, detd);

    vector<Marker> gms;
    fill(detd, gms);

    //---- TODO: group search table
    for(auto& g : mc.grps_)
    {
        //---- pose estimate
        for(auto& m : gms)
        {
            auto& ids = g.ids;
            auto it = ids.find(m.id);
            if(it==ids.end())continue;

            ok &= m.pose_est(cfg_.camc, g.w);
            ms.push_back(m);
        }
        
    }
    //--- detect boards
    for(auto pc : cfg_.mcfg.boards_)
    {
        auto& bc = reinterpret_cast<BrdCfgImp&>(*pc);
        auto p = mkSp<Board>();
        auto& brd = *p;
        if(!bc.det(detd, camc, brd.pose))
            continue;
        brd.p_cfg = pc;
        result_.boards.push_back(p);
    }
    //----- show
    if(mc.en_imo)
       result_.p_imo = gen_imo(im);
    
   
    return true;
}


//-----------
Sp<Img> Marker::PoseEstimator::gen_imo(const Img& im)const
{
    auto p_imo = im.copy();
    auto& imo = *p_imo;
    auto& ms = result_.ms;
    auto& camc = cfg_.camc;
    
    //----
    auto sz = camc.sz;
    float dw = sz.w *0.05;
    float dh = sz.h *0.05;
    // Note: axis length half of marker width
    Color ct{0,200,255};
    Color cp{255,0,0};
    Color cb{100,200,255};
    for(auto& m : ms)
    {
        imo.draw(camc, {m.pose, m.w*0.5, 2});
        //--- left/top corner (px0)
        vec2 v0 = m.ps[0];
        imo.draw({v0}, cp, 10);
        
        // id
        {
            stringstream s; s << "id=" << m.id;
            imo.draw(s.str(), toPx(v0), ct);
        }
        // t
        if(0)
        {
            string s = "t="+ egn::str(m.pose.t, 2);
            imo.draw(s, toPx(v0) + Px(dw,dh), ct);
        }

    }
    //--- draw boards
    for(auto& p : result_.boards)
    {
        auto& b = *p;
        auto pc = p->p_cfg;
        assert(pc!=nullptr);
        //----
        auto& Tcw = b.pose;
        auto& Twb = pc->T;
        Pose Tcb = Tcw*Twb;
        //---- axis
        imo.draw(camc, {Tcw, 0.3, 2});

        //--- board box
        auto lns = pc->box.cube().edges();
        for(auto& l : lns) 
            l.trans(Tcb);
        imo.draw(camc, lns, cb, 2);

        //--- txt        
        vec2 vc = camc.proj(Tcb.t);
        imo.draw(pc->sName, toPx(vc), cb);
        string s = "t="+ egn::str(Tcb.t, 2);
        imo.draw(s, toPx(vc)+Px(dw,dh), cb);
        
    }

    return p_imo;
}


//-----------
bool Marker::pose_est(const CamCfg& cc, double wid)
{
    w = wid;

    cv::Mat Kc,Dc;
    eigen2cv(cc.K, Kc);
    eigen2cv(cc.D, Dc);
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
// TODO: deprecated , replace by avg pose
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

//-------
Sp<const Board> MarkerPE::Result::
                nearstBoard(const string& s)const
{
    double dmin = -1;
    Sp<const Board> pr = nullptr;
    for(auto p: boards)
    {
        auto pc = p->p_cfg;
        if(pc==nullptr || pc->sName!=s)
            continue;
        double d = p->pose.t.squaredNorm();
        if(dmin >=0 && d > dmin)continue;
        dmin = d;
        pr = p;

    }
    return pr;
}
//----
bool MarkerPE::Result::find_pose(
    const vector<string> sBoards, Pose& Tcb)const
{
    // Note: here 'w' frame mean 
    // multi-board frame, for easier convention.
    // Same as 'b' frame as output boards frame.

    //--- find board "platte"
    vector<Pose> Tcws;
    for(auto& s : sBoards)
    {
        auto pb = nearstBoard(s);
        if(pb==nullptr) continue;
        Pose Tcw = pb->pose;
        //---- origin
        //Pose Tcb = pb->pose; // detected pose
        //Pose Tcw = Tcb * r.Tbw;
        Tcws.push_back(Tcw);
    }    
    //-----
    if(Tcws.size()==0)
    {
        log_e("   Could not find marker board '");
        return false;
    }

    //--- fit average
    Tcb = Pose::avg(Tcws);

    return true;
}

