#include "vsn/vsnLibCv.h"
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "json/json.h"



using namespace vsn;

//---- Factory
Sp<StereoVO> StereoVO::create()
{
    return mkSp<StereoVOcv>();
}

//-----------
string StereoVO::Cfg::str()const
{
    stringstream s;
    s << "{stereo:{";
    s << "baseline:" << baseline;
    s << "}}" << endl;
    return s.str();
}

//-----------
bool StereoVO::Cfg::load(const string& sf)
{
    log_i("Load StereoVO cfg :'"+sf+"'");
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
        auto& js = jd["stereo"];
        baseline = js["baseline"].asDouble();

        auto& jo = js["odometry"];
        odom.mode = jo["mode"].asInt();
        odom.z_TH = jo["z_TH"].asDouble();

        auto& jf = js["feature"];
        feature.Nf = jf["Nf"].asInt();

        auto& jpc = js["point_cloud"];
        pntCloud.z_TH = jpc["z_TH"].asDouble();

        auto& jr = js["run"];
        run.bShow = jr["show"].asBool();
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

//------------------- StereoVOcv --------------
//-----------
bool StereoVOcv::Frm::find(int i, bool bLeft, MPnt& mpnt)const
{
    if(p_fm==nullptr) return false;
    auto& fm = *p_fm;
    auto& md = fm.data_.md;
    auto& i_mi = (bLeft)?md.i1_mi : md.i2_mi;
    auto it = i_mi.find(i);
    if(it==i_mi.end())return false;
    int mi = it->second;
    if(mi >= mpnts.size()) return false;
    mpnt = mpnts[mi];
    return true;
}
//-----------
bool StereoVOcv::Frm::at(int mi, MPnt& mpnt)const
{
    if(mi<0 || mi>= mpnts.size())
        return false;
    mpnt = mpnts[mi];
    return true;
}
 
//-----------
bool StereoVOcv::onImg(const Img& im1,  
                       const Img& im2)
{
    auto& camc = cfg_.camc;

    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);
    imc1.undistort(camc);
    imc1.undistort(camc);

    bool ok = true;
    //---- do feature matching of L/R
    auto p_fm = mkSp<FeatureMatchCv>();
    auto& fm = *p_fm;
    fm.cfg_.bShow = cfg_.bShow;
    fm.cfg_.N = cfg_.feature.Nf;
    ok &= fm.onImg(im1, im2);

    //---- construct frm
    auto p_frm = mkSp<Frm>();
    auto& frm = *p_frm;
    frm.p_fm = p_fm;

    //---- trangulate feature points.
    ok &= triangulate(fm, frm.mpnts);
    
    //---- do odometry
    auto p_frmp = data_.p_frm_prev;
    if(p_frmp!=nullptr)
        odometry(*p_frmp, *p_frm);

    //---- save to previous frm
    data_.p_frm_prev = p_frm;
    return ok;
}
//-----------------
bool StereoVOcv::triangulate(const FeatureMatchCv& fm,
                             vector<MPnt>& mpnts)const
{
    bool ok = true;
    stringstream s;

    auto& camc = cfg_.camc;
    mpnts.clear();
    // ( inner arry for each image)
    vector<cv::Point2d> Qs1, Qs2; // for calib triangulation
    //vector<cv::Point2d> Qs;// for sfm triangulation
    auto& ms = fm.FeatureMatch::data_.ms;
    int N = ms.size();
    for(auto& m : ms)
    {
        auto Q1 = ocv::toCv(m.p1);
        auto Q2 = ocv::toCv(m.p2);
        Qs1.push_back(Q1);
        Qs2.push_back(Q2);
    //    Qs.push_back(Q1);
    //    Qs.push_back(Q2);
    }
    //---- projection matrix P = K*T
    // We have 2 cameras.
    //  ( Note, projMat for cam coordinate, 
    //     so reverse transform)
    double b = cfg_.baseline;
    cv::Mat T1 = (cv::Mat_<double>(3,4) << 
            1, 0, 0,  b*0.5,
            0, 1, 0,  0,
            0, 0, 1,  0);
    cv::Mat T2 = (cv::Mat_<double>(3,4) << 
            1, 0, 0,  -b*0.5,
            0, 1, 0,  0,
            0, 0, 1,  0);

    cv::Mat K; cv::eigen2cv(camc.K, K); 
    cv::Mat Ps;
    cv::Mat P1 = K*T1;
    cv::Mat P2 = K*T2;

//   1) calib3d triangulation function
    cv::triangulatePoints(P1, P2, Qs1, Qs2, Ps);

 //  2) sfm triangulation function
 //   vector<cv::Mat> pmats;
 //   pmats.push_back(P1);
 //   pmats.push_back(P2);
 //   cv::sfm::triangulatePoints(Qs, pmats, Ps);
    s << "Triangulate pnts: " << N << endl;
    
    //---- De-homoge and fill triangulation result
    //s << "  Triangulate: " << endl;
    for(int i=0;i<N;i++)
    {
        vec4 h = ocv::toVec4(Ps.col(i));
        vec3 v; 
        if(!egn::normalize(h, v))
           v << 0,0,0;
        MPnt p;   
        p.mi = i;     
        p.Pt = cv::Point3f(v[0], v[1], v[2]);
        mpnts.push_back(p);
    //    s << "Pair:" << Qs1[i] << " | " << Qs2[i] << " => ";
      //  s << "(" << v.transpose() << ")" << endl;
    }
    log_d(s.str());
    return ok;
}


//-----------------
bool StereoVOcv::odometry(const Frm& frm1,
                          const Frm& frm2)
{
    stringstream s;
    auto& odomc = cfg_.odom;
    //---- cam motion for left/right
    cv::Mat rL,rR,tL,tR;
    set<int> inliers;
    bool okL = solve_2d3d(frm1, frm2, true,  rL, tL, inliers);
    bool okR = solve_2d3d(frm1, frm2, false, rR, tR, inliers);
    if(!(okL | okR)) {
        log_e("stereo odometry failed");
        return false;
    }
    //---- Average or select
    cv::Mat rc = (okL && okR)? (rL + rR)*0.5 :
                okL ? rL : rR;
    cv::Mat tc =  (okL && okR)? (tL + tR)*0.5 :
                okL ? tL : tR;
    cv::Mat Rc;
    cv::Rodrigues(rc, Rc); 

    //------
    cv::Mat ec = rc * 180.0/M_PI; // to degree
    cv::Mat ec1, tc1; 
    cv::transpose(ec, ec1); cv::transpose(tc, tc1);
    s << "  Relative motion:  ";
    s << "ec=" << ec1 << ", tc=" << tc1 << endl; 
    // R/t is relative motion from frm1 to frm2
    //---- Update R/t global
    auto& odom = StereoVO::data_.odom;
    mat3 Re; cv::cv2eigen(Rc, Re);
    vec3 te; cv::cv2eigen(tc, te);
    auto& Rw = odom.Rw;
    auto& tw = odom.tw;
    auto& ew = odom.ew;
    tw = tw + Rw * te;
    Rw = Rw * Re;
    //---- Euler pose
    cv::Mat Rwc, rwc;
    cv::eigen2cv(Rw, Rwc);
    cv::Rodrigues(Rwc, rwc); 
    cv::Mat ewc = rwc*180.0/M_PI; // to degree
    cv::cv2eigen(ewc, ew);
   
    //---- calc global points and fill
    auto p_frm = mkSp<StereoVO::Frm>();
    StereoVO::data_.p_frm = p_frm;
    calc_pnts(frm2, inliers, p_frm->Pws);
    
    //---
    s << "  Global odom: ew=" << ew.transpose() 
        << ", tw=" << tw.transpose() << endl;
    log_d(s.str());

    return true;    
}

//-----------------
bool StereoVOcv::solve_2d3d(const Frm& frm1,
                            const Frm& frm2,
                            bool bLeft,
                            cv::Mat& r, cv::Mat& t,
                            set<int>& inliers)const
{
    auto& odomc = cfg_.odom;

//  auto pm = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    auto& fm1 = *frm1.p_fm;
    auto& fm2 = *frm2.p_fm;

    FeatureMatchCv fmL, fmR;
    auto& fmd1 = fm1.data_;
    auto& fmd2 = fm2.data_;
    stringstream s;

    //---- 
    // mdL is match of L channel 
    // mdR is match of R channel
    FeatureMatchCv::MatchDt mdL, mdR;
    fmL.match(fmd1.fs1, fmd2.fs1, mdL);
    fmR.match(fmd1.fs2, fmd2.fs2, mdR);

    //---- to be filled
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;

    //---- Left odometry
  //  auto& i_mi = mdL.i1_mi;
    auto& md = bLeft?mdL:mdR;
    vector<int> mi_ary; // save index
    for(auto& m : md.dms)
    {
        int i1 = m.queryIdx; // fi frm1
        int i2 = m.trainIdx; // fi frm2
        //   search frm2 for 3d pnt
        MPnt mpnt;
        if(!frm2.find(i2, bLeft, mpnt))
            continue;
        // got mpnt is match pnt also
        //   of L/R in frm1.
        // Which has been triangulated.
        auto P = (odomc.mode==1)?
            mpnt.Pt : mpnt.Pd;
        if(P.z > odomc.z_TH)
            continue;
        mi_ary.push_back(mpnt.mi);
        pts_3d.push_back(P);
        // Find 2d pnt of previous frm
        auto& fmdQ = fmd1;
        auto& fs = bLeft?fmdQ.fs1 : fmdQ.fs2;
        auto Q = fs.pnts[i1].pt;
        pts_2d.push_back(Q);
    }
    //--- dbg
    int N = pts_2d.size();
    

    //---- do solving
    //---- solve PnP
    cv::Mat inlrs;
    cv::Mat K; 
    cv::eigen2cv(cfg_.camc.K, K);
    if(!cv::solvePnPRansac(pts_3d, pts_2d, K, cv::Mat(), r, t, inlrs))
    {
        log_e("  solvePnPRansac() failed");
        return false;
    }
    //-------
    // ref : https://answers.opencv.org/question/196562/solvepnpransac-getting-inliers-from-the-2d-and-3d-points/
    int Ni = inlrs.rows;
    s << "  solvePnP() inliers: " << Ni << " of " << N << endl;
    //-------
    cv::Mat e = r*180.0/M_PI; // to degree
    cv::Mat e1,t1; 
    cv::transpose(e, e1); cv::transpose(t, t1);
    s << " cam " << (bLeft?"L":"R") << " motion: ";
    s << "e=" << e1 << ", t=" << t1 << endl; 
    // fill inliers
    for (int i = 0; i < inlrs.rows; i++)
    {
        int k = inlrs.at<int>(i, 0);
        cv::Point3f Pc = pts_3d[k];
        int mi = mi_ary[k];
        inliers.insert(mi);
        /*
        s << "  2d/3d:(" << pts_2d[k] 
            << ") -> ("  << pts_3d[k]  
            << ")" << endl; 
            */
    }
    // R/t is relative motion from frm1 to frm2
    log_d(s.str());
    return true;
    
}
//-----------
void StereoVOcv::calc_pnts(const Frm& frmc,
                           const set<int>& mi_ary,
                           vec3s& Ps)const
{
    auto& odomc = cfg_.odom;
    auto& pcldc = cfg_.pntCloud;
    auto& odom = StereoVO::data_.odom;

    auto& Rw = odom.Rw;
    auto& tw = odom.tw;
    //---- calc global points with inliers
    for(auto& mi : mi_ary)
    {
        MPnt mpnt;
        if(!frmc.at(mi, mpnt))
            continue;
        auto Pc = (odomc.mode==1)?
            mpnt.Pt : mpnt.Pd; 
        //--- Threshhold and check
        if(Pc.z > pcldc.z_TH)continue;
        if(Pc.z <0)continue;
        //----
        vec3 Pe; Pe << Pc.x, Pc.y, Pc.z;

        auto P = Rw*Pe + tw;
        Ps.push_back(P);
    }
}

//-----------
bool StereoVOcv::genDepth(const Img& im1,  
                          const Img& im2)
{
    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);

    bool ok = true;
   
    //---------------
    // Setting Ref : 
    //   https://jayrambhia.com/blog/disparity-mpas
    //
    /*
        sgbm.SADWindowSize = 5;
        sgbm.numberOfDisparities = 192;
        sgbm.preFilterCap = 4;
        sgbm.minDisparity = -64;
        sgbm.uniquenessRatio = 1;
        sgbm.speckleWindowSize = 150;
        sgbm.speckleRange = 2;
        sgbm.disp12MaxDiff = 10;
        sgbm.fullDP = false;
        sgbm.P1 = 600;
        sgbm.P2 = 2400;
    */
    /* setting (1)
    auto p_sgbm = cv::StereoSGBM::create(
        -64, //  int minDisparity = 0, 
        16, // int numDisparities = 16, 
        3, // int blockSize = 3,
        600,  // int P1 = 0, 
        2400, // int P2 = 0, 
        10, // int disp12MaxDiff = 0,
        4, // int preFilterCap = 0, 
        1,// int uniquenessRatio = 0,
        150, // int speckleWindowSize = 0, 
        2 // int speckleRange = 0,
        // int mode = StereoSGBM::MODE_SGBM
    );
    */
    // Setting (2)
    auto p_sgbm =  cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); // tested parameters
    auto& sgbm = *p_sgbm;

    //---------------
    cv::Mat im_disp, im_disp2;
    sgbm.compute(imc1.im_, imc2.im_, im_disp);

    //---- display
    cv::normalize(im_disp, im_disp2, 0, 255, cv::NORM_MINMAX, CV_8U);
    //im_disp2 = im_disp*10;

    StereoVO::data_.p_imd_ = mkSp<ocv::ImgCv>(im_disp);

    if(cfg_.bShow)
    {
        string sName = "depth";
        cv::namedWindow(sName, cv::WINDOW_KEEPRATIO);
        imshow(sName, im_disp2);
        cv::waitKey(10);
    }
    return ok;
}
