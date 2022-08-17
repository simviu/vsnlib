#include <opencv2/features2d/features2d.hpp>

#include "vsn/vsnLib.h"
#include "vsn/vsnLibCv.h"


using namespace vsn;
using namespace ut;
using namespace cv;

//--- Factory
Sp<FeatureMatch> FeatureMatch::create()
{
    return mkSp<FeatureMatchCv>();
}

//-------
bool FeatureMatchCv::onImg(const Img& im1,
                           const Img& im2)
{
    auto& imc1 = reinterpret_cast<const ocv::ImgCv*>(&im1)->im_;
    auto& imc2 = reinterpret_cast<const ocv::ImgCv*>(&im2)->im_;
    //--- ref : SlamBook ch7
    //    https://github.com/gaoxiang12/slambook
    //
    //  
    //-- 初始化
    //Mat descriptors_1, descriptors_2;
    //std::vector<KeyPoint> keypoints_1;
    //std::vector<KeyPoint> keypoints_2;
    auto& descriptors_1 = data_.kps1.desc;
    auto& descriptors_2 = data_.kps2.desc;
    auto& keypoints_1 = data_.kps1.pnts;
    auto& keypoints_2 = data_.kps2.pnts;
    
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create(cfg_.N);
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( imc1,keypoints_1 );
    detector->detect ( imc2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( imc1, keypoints_1, descriptors_1 );
    descriptor->compute ( imc2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    //vector<DMatch> match;
    auto& dms = data_.matches;
    dms.clear();
    match(data_.kps1, data_.kps2, dms);

    //---- fill result
    auto& ms = FeatureMatch::data_.ms;    
    ms.clear();
    for(auto& m : dms)
    {
        auto& kp1 = keypoints_1[m.queryIdx].pt;
        auto& kp2 = keypoints_2[m.trainIdx].pt;
        Match fm;
        fm.p1 << kp1.x, kp1.y;
        fm.p2 << kp2.x, kp2.y;
        ms.push_back(fm);       
    }

    //---- dbg show img
    Mat img_match;
    drawMatches(imc1, keypoints_1, imc2, keypoints_2, dms, img_match);
    if (cfg_.bShow)
    {
        string sName = "featureMatch";
        cv::namedWindow(sName, cv::WINDOW_KEEPRATIO);
        imshow(sName, img_match);
        waitKey(10);
    }
    return true;        
}
//-------
bool FeatureMatchCv::match(
            const KeyPnts& kps1,
            const KeyPnts& kps2,
            vector<cv::DMatch>& dms)const
{
    dms.clear();
    // BFMatcher matcher ( NORM_HAMMING );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    vector<cv::DMatch> dms_pre;
    matcher->match ( kps1.desc, kps2.desc, dms_pre );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < kps1.desc.rows; i++ )
    {
        double dist = dms_pre[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

  //  printf ( "-- Max dist : %f \n", max_dist );
  //  printf ( "-- Min dist : %f \n", min_dist );
    
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < kps1.desc.rows; i++ )
    {
        auto& m = dms_pre[i];
        if ( m.distance > max ( 2*min_dist, cfg_.distTH ) )
            continue;
        int i1 = m.queryIdx;
        int i2 = m.trainIdx;

        dms.push_back(m);
    }
    return true;
}


