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
    return true;
}

bool Server::open(const string& sf)
{
    return true;

}

bool Server::open(int cam_id)
{
    return true;

}

void Server::push(Sp<Img> p)
{

}
