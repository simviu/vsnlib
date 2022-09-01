/*
   Author: Sherman Chen
   Create Time: 2022-09-01
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/pcl_utils.h"
#include "json/json.h"

using namespace vsn;
        
 Points::Points()
 {
     p_data_ = mkSp<DataPcl>();
 }

DataPcl& DataPcl::cast(Points::Data& d)
{ 
    return reinterpret_cast<DataPcl&>(d); 
}

//---------------

bool Points::load(const string& sf)
{
    return true;
}
bool Points::save(const string& sf)
{
    auto p = DataPcl::cast(*p_data_).getCloud();
    pcl::io::savePCDFileASCII (sf, p);

    return true;

}
void Points::show()const
{
    return true;

}
