#include <opencv2/features2d/features2d.hpp>

#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"


using namespace vsn;
using namespace ut;
using namespace cv;
//-------
bool Feature::match(const Img& im1,
                    const Img& im2,
                    vector<Match>& ms)
{
    auto& imc1 = reinterpret_cast<const ocv::ImgCv*>(&im1)->im_;
    auto& imc2 = reinterpret_cast<const ocv::ImgCv*>(&im2)->im_;
    //--- ref : SlamBook ch7
    //    https://github.com/gaoxiang12/slambook
    //
    //  
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    std::vector<KeyPoint> keypoints_1;
    std::vector<KeyPoint> keypoints_2;

    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( imc1,keypoints_1 );
    detector->detect ( imc2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( imc1, keypoints_1, descriptors_1 );
    descriptor->compute ( imc2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        auto& m = match[i];
        if ( m.distance <= max ( 2*min_dist, 30.0 ) )
        {
            auto& kp1 = keypoints_1[m.queryIdx].pt;
            auto& kp2 = keypoints_2[m.trainIdx].pt;

            //matches.push_back ( match[i] );
        }
    }
    return true;        
}



