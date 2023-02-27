/*
   Author: Sherman Chen
   Create Time: 2022-09-01
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLibCv.h"
#include "vsn/pcl_utils.h"
#include "json/json.h"


using namespace vsn;
namespace{
    //----- utils 
    pclu::Pnt toPcl(const Points::Pnt& in)
    { 
        auto& c = in.c;
        auto& v = in.p;
        std::uint32_t rgb = (static_cast<std::uint32_t>(c.r) << 16 |
                    static_cast<std::uint32_t>(c.g) << 8 | static_cast<std::uint32_t>(c.b));
        pclu::Pnt p;
        p.rgb = *reinterpret_cast<float*>(&rgb);
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    }
    //------------
    // Points::Data Imp
    //------------
    struct DataImp : public Points::Data
    {
        pclu::PCloud::Ptr p_cloud_ =
           pclu::PCloud::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        auto raw(){ return p_cloud_; }
        auto raw()const{ return p_cloud_; }
    };
    //----
    pclu::PCloud::ConstPtr getRaw(const Points& d)
    {   
        auto p = d.getData();
        auto& di = reinterpret_cast<const DataImp&>(*p);
        return di.raw();
    }
    pclu::PCloud::Ptr getRaw(Points& d)
    {   
        auto p = d.getData();
        auto& di = reinterpret_cast<DataImp&>(*p);
        return di.raw();
    }
   
   
    //---------------
    // Visualization
    //---------------
    using  PclVis = pcl::visualization::PCLVisualizer; 
    class VisImp : public Points::Vis{
    public:
        VisImp(const Cfg& c);
        auto raw(){ return p_pcl_vis_; }
    protected:
        PclVis::Ptr p_pcl_vis_ = nullptr;
    };
    PclVis::Ptr getRaw(Points::Vis& v)
    { return (reinterpret_cast<VisImp&>(v)).raw(); }
    //-----
    VisImp::VisImp(const Cfg& c)
    {
        auto cb = c.bk_color;
        auto p = PclVis::Ptr(new PclVis(c.sName));
        p->setBackgroundColor (cb.r/255.0, cb.g/255.0, cb.b/255.0);
        p->addCoordinateSystem (c.axisL);
        p->initCameraParameters();
        p_pcl_vis_ = p;
    }
    
}
//----
Points::Points()
{
    p_data_ = mkSp<DataImp>();
}

//----
void Points::Vis::add(const Points& pd, 
                     const string& sName,
                     float pnt_sz)
{
    auto p_vi = getRaw(*this);
    auto& vi = *p_vi;
    auto p = getRaw(pd);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(p);
    vi.addPointCloud<pcl::PointXYZRGB> (p, rgb, sName);
    vi.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pnt_sz, sName);
}
//----
bool Points::Vis::spin()
{
    auto p = getRaw(*this);
    p->spinOnce (100);
    return !(p->wasStopped());
}
//----
void Points::Vis::clear()
{
    auto p = getRaw(*this);
    p->removeAllPointClouds();
}
//----- factory
Sp<Points::Vis> Points::Vis::create(const Cfg& c)
{ return mkSp<VisImp>(c); }

//---------------
// 
//---------------
void Points::clear()
{
    auto pc = getRaw(*this);
    pc->points.clear();
 
}
//---
int Points::num()const
{
    auto& di = reinterpret_cast<DataImp&>(*p_data_);
    return di.raw()->points.size();
}
//---- info
string Points::Stats::str()const
{
    string s = "box:";
    s += "{" + box.str()+"}";
    return s;
}
//----
string Points::info()const
{
    stringstream s;
    s << "N:" << num() << ", ";
    s << "stats:{" << stats_.str() << "}";
    return s.str();
}

//---
void Points::add(const Pnt& p)
{
    auto pc = getRaw(*this);
    pc->points.push_back (toPcl(p));
    pc->width = pc->size();
    pc->height = 1;
    stats_.box.upd(p.p);

}

//---------------
bool Points::load(const string& sf)
{
    auto p = getRaw(*this);
    if (pcl::io::loadPCDFile(sf, *p) == -1) 
    {
        log_ef(sf);
        return false;
    }
    log_i("Load Point Cloud OK:'"+sf+"'");
    return true;
}
//----
bool Points::save(const string& sf)const
{
    auto p = getRaw(*this);
    int r = pcl::io::savePCDFileASCII(sf, *p);
    if(r==-1)
    {
        log_ef(sf);
        return false;
    }

    log_i("Load Point Cloud OK:'"+sf+"'");

    return true;
}
//----
void Points::gen_cylinder()
{
    Color c{255, 15, 15};
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            Pnt p;
            p.p << 0.5 * std::cos (pcl::deg2rad(angle)), sinf (pcl::deg2rad(angle)), z;
            p.c = c;
            add(p);
        }
        if (z < 0.0) { c.r -= 12;  c.g += 12; }
        else { c.g -= 12;  c.b += 12; }
    }

}
//-------
// Ref: slambook
void Points::filter_stats(float meanK, float devTh)
{
    auto& d = reinterpret_cast<DataImp&>(*p_data_);
    pclu::PCloud::Ptr p ( new pclu::PCloud );
    pcl::StatisticalOutlierRemoval<pclu::Pnt> f;
    f.setMeanK(meanK);
    f.setStddevMulThresh(devTh);
    f.setInputCloud(d.p_cloud_);
    f.filter( *p );
    d.p_cloud_ = p;
    
}
//-------
void Points::filter_voxel(float reso)
{
    auto& d = reinterpret_cast<DataImp&>(*p_data_);

   // voxel filter 
    pcl::VoxelGrid<pclu::Pnt> f; 
    float r = reso;
    f.setLeafSize( r, r, r );       // resolution 
    pclu::PCloud::Ptr tmp ( new pclu::PCloud );
    f.setInputCloud( d.p_cloud_ );
    f.filter( *tmp );
    tmp->swap( *d.p_cloud_ );
    
}


