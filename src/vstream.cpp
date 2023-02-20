/*
   Author: Sherman Chen
   Create Time: 2023-02-19
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"

using namespace vsn;
using namespace vstream;


bool Server::init(int port)
{
    log_i("Start vstream server at port "+to_string(port));
    bool ok = svr_.start(port);
    if(!ok) log_e(" vstream server failed");
    return ok;
}
//----
bool Server::open(const string& sf)
{
    return true;

}
//----
bool Server::open(int cam_id)
{
    p_video_ = Video::create(cam_id);
    if(p_video_!=nullptr)
    {
        log_i("vstream server openned camera:"+to_string(cam_id));
        return true;
    }
    log_e("vstream server failed to open camera:"+to_strimg(cam_id));
    return false;

}

void Server::push(Sp<Img> p)
{

}
