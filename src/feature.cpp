#include <opencv2/features2d/features2d.hpp>

#include "vsn/vsnLib.h"
#include "vsn/vsnLibCv.h"


using namespace vsn;
using namespace ut;
using namespace cv;
//-------
bool FeatureMatch::onImg(const Img& im1,
                         const Img& im2)
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
    Ptr<FeatureDetector> detector = ORB::create(cfg_.N);
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

  //  printf ( "-- Max dist : %f \n", max_dist );
  //  printf ( "-- Min dist : %f \n", min_dist );

    auto& ms = result_.ms;
    ms.clear();
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        auto& m = match[i];
        if ( m.distance <= max ( 2*min_dist, cfg_.distTH ) )
        {
            auto& kp1 = keypoints_1[m.queryIdx].pt;
            auto& kp2 = keypoints_2[m.trainIdx].pt;

            //matches.push_back ( match[i] );
            Match fm;
            fm.p1 << kp1.x, kp1.y;
            fm.p2 << kp2.x, kp2.y;
            ms.push_back(fm);
        }
    }
    //---- dbg show img
    Mat img_match;
    drawMatches(imc1, keypoints_1, imc2, keypoints_2, match, img_match);
    if (cfg_.bShow)
    {
        string sName = "featureMatch";
        cv::namedWindow(sName, cv::WINDOW_KEEPRATIO);
        imshow(sName, img_match);
        waitKey(10);
    }
    return true;        
}



