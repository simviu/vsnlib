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
    //------------
    // Points::Data Imp
    //------------
    struct DataImp : public Points::Data
    {
        pclu::PCloud::Ptr p_cloud_ =
           pclu::PCloud::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        //static DataPcl& cast(const Points::Data& d);
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
bool Points::Vis::spin()
{
    auto p = getRaw(*this);
    p->spinOnce (100);
    return !(p->wasStopped());
}

//----- factory
Sp<Points::Vis> Points::Vis::create(const Cfg& c)
{ return mkSp<VisImp>(c); }


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
    auto p = getRaw(*this);
    std::uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZRGB pnt;
            pnt.x = 0.5 * std::cos (pcl::deg2rad(angle));
            pnt.y = sinf (pcl::deg2rad(angle));
            pnt.z = z;
            std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                    static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
            pnt.rgb = *reinterpret_cast<float*>(&rgb);
            p->points.push_back (pnt);
        }
        if (z < 0.0) { r -= 12;  g += 12; }
        else { g -= 12;  b += 12; }
    }
    p->width = p->size ();
    p->height = 1;

}

