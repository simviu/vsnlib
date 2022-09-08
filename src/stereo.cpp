#include "vsn/vsnLib.h"
#include "json/json.h"

using namespace vsn;

//----
namespace{
    const struct{
        string sf_pnts_spar = "pnts_sparse.xyz";
        string sf_Tw = "Tw.txt";
    }lcfg_;
    //---- utils
    string gen_Tw3x4_line(const mat3& Rw, 
                          const vec3& tw, 
                          int idx)
    {
        stringstream s;
        s.precision(16);
        s << std::fixed;
        s << idx ; // current frame index

        mat3x4 Tw;
        Tw << Rw, tw;
        for(int i=0; i<Tw.rows(); i++)
            for(int j=0; j<Tw.cols(); j++)
                s << " " << Tw(i, j);
        s << endl;
        string sr = s.str();
        return sr;
    }
    //---- load SGBM json cfg
    bool decode(const Json::Value& j,
                StereoVO::DisparityCfg::SGBM& c)
    {
        c.minDisparity = j["minDisparity"].asInt();
        c.numDisparities = j["numDisparities"].asInt();
        c.blockSize = j["blockSize"].asInt();
        c.P1 = j["P1"].asInt();
        c.P2 = j["P2"].asInt();
        c.disp12MaxDiff = j["disp12MaxDiff"].asInt();
        c.preFilterCap = j["preFilterCap"].asInt();
        c.uniquenessRatio = j["uniquenessRatio"].asInt();
        c.speckleWindowSize = j["speckleWindowSize"].asInt();
        c.speckleRange = j["speckleRange"].asInt();

        //---- filter
        //----
        auto& jw = j["wls_filter"];
        auto& wls = c.wls_filter;
        wls.en = jw["en"].asBool();
        wls.lambda = jw["lambda"].asFloat();
        wls.sigma  = jw["sigma"].asFloat();

        return true;
    
    }
    //----
    bool decode(const Json::Value& j, 
                StereoVO::DisparityCfg& c)
    {
        bool ok = true;
        ok &= decode(j["sgbm"], c.sgbm);
        
        //----
        c.vis_mul = j["vis_mul"].asFloat();

        return ok;
    } 
    
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
        //--- disparity cfg
        decode(js["disparity"], dispar);
        //---- point cloud
        {
            auto& jpc = js["point_cloud"];
            auto& pc = pntCloud;
            pc.z_TH = jpc["z_TH"].asDouble();
            auto& jfc = js["filter"];
            auto& fc = pc.filter;
            fc.en = jfc["en"].asBool();
            fc.meanK = jfc["meanK"].asFloat();
            fc.devTh = jfc["devTh"].asFloat();
            fc.voxel_res = jfc["voxel_res"].asFloat();
        }

        //---- run
        auto& jr = js["run"];
        run.bShow   = jr["show"].asBool();
        run.enDense = jr["enDense"].asBool();
        run.enDepth = jr["enDepth"].asBool();
        run.enWr    = jr["enWr"].asBool();
        
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
//-----
bool StereoVO::Data::Wr::open()
{
    ofs_pnts_spar.open(lcfg_.sf_pnts_spar);
    bool ok1 = ofs_pnts_spar.is_open();
    if(!ok1)
        log_ef(lcfg_.sf_pnts_spar);
    //----
    ofs_Tw.open(lcfg_.sf_Tw);
    bool ok2 = ofs_Tw.is_open();
    if(!ok2)
        log_ef(lcfg_.sf_Tw);

    return ok1 && ok2;
}
//----
void StereoVO::Data::Wr::close()
{ 
    ofs_pnts_spar.close(); 
    ofs_Tw.close(); 
}

//------------
bool StereoVO::Data::wrData()
{
    auto& fi = frmIdx;
    //--- write Tw
    {
        auto& f = wr.ofs_Tw;
        auto& Rw = odom.Rw;
        auto& tw = odom.tw;
        if(f.is_open())
            f << gen_Tw3x4_line(Rw, tw, fi);
    }

    //--- write points
    if(p_frm!=nullptr)
    {
        auto& f = wr.ofs_pnts_spar;
        if(f.is_open())
            for(auto& P : p_frm->Pws)
                f << P.x() << " " << P.y() << " " << P.z() << endl;
    }
    return true;
}

