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
/*
DataPcl& DataPcl::cast(const Points::Data& d)
{ 
    return reinterpret_cast<DataPcl&>(d); 
}
*/
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
//----- factory
Sp<Points::Vis> Points::Vis::create(const Cfg& c)
{ return mkSp<VisImp>(c); }

//---------------

bool Points::load(const string& sf)
{
    return true;
}
bool Points::save(const string& sf)
{
    auto p = getRaw(*this);
    pcl::io::savePCDFileASCII (sf, *p);

    return true;

}


