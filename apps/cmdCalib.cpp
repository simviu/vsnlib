/*
   Author: Sherman Chen
   Create Time: 2022-09-07
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/vsnTool.h"
#include "vsn/vsnLibCv.h"


#include <opencv2/ccalib/multicalib.hpp>

using namespace app;

//----
namespace{
    struct LCfg{
        float contrast_scl = 1.0;
    }; LCfg lc_;

    //---------------------
    //  stereo calib step1
    //---------------------
    bool stereo_calib_step1(const string& sDir)
    {
        // Defining the dimensions of checkerboard
        int CHECKERBOARD[2]{3,4}; 

        // Creating vector to store vectors of 3D points for each checkerboard image
        std::vector<std::vector<cv::Point3f> > objpoints;

        // Creating vector to store vectors of 2D points for each checkerboard image
        std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for(int i{0}; i<CHECKERBOARD[1]; i++)
        {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
        }

        // Extracting path of individual image stored in a given directory
        std::vector<cv::String> imagesL, imagesR;
        // Path of the folder containing checkerboard images
        std::string pathL = sDir + "/L/*.jpg";
        std::string pathR = sDir + "/R/*.jpg";

        cv::glob(pathL, imagesL);
        cv::glob(pathR, imagesR);

        cv::Mat frameL, frameR, grayL, grayR;
        // vector to store the pixel coordinates of detected checker board corners 
        std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
        bool successL, successR;
        int N = imagesL.size();
        assert(N<=imagesR.size());
        log_e(to_string(N) + " images L/R pair found in:"+sDir);
        // Looping over all the images in the directory
        for(int i{0}; i<N; i++)
        {
            frameL = cv::imread(imagesL[i]);
            cv::cvtColor(frameL,grayL,cv::COLOR_BGR2GRAY);

            frameR = cv::imread(imagesR[i]);
            cv::cvtColor(frameR,grayR,cv::COLOR_BGR2GRAY);

            //---- enhance
            grayL = grayL * lc_.contrast_scl;
            grayR = grayR * lc_.contrast_scl;
            // Finding checker board corners
            // If desired number of corners are found in the image then success = true  
            int flag =  cv::CALIB_CB_ADAPTIVE_THRESH + 
                        cv::CALIB_CB_NORMALIZE_IMAGE + 
                        cv::CALIB_CB_EXHAUSTIVE;
            successL = cv::findChessboardCorners(
                grayL,
                cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
                corner_ptsL, flag);
                // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

            successR = cv::findChessboardCorners(
                grayR,
                cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
                corner_ptsR, flag);
                // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
            /*
                * If desired number of corner are detected,
                * we refine the pixel coordinates and display 
                * them on the images of checker board
            */
            if((successL) && (successR))
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

                // refining pixel coordinates for given 2d points.
                cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);
                cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);

                // Displaying the detected corner points on the checker board
                cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsL,successL);
                cv::drawChessboardCorners(frameR, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsR,successR);

                objpoints.push_back(objp);
                imgpointsL.push_back(corner_ptsL);
                imgpointsR.push_back(corner_ptsR);
            }

            cv::imshow("ImageL",frameL);
            cv::imshow("ImageR",frameR);
            cv::imshow("GrayL",grayL);
            cv::imshow("GrayR",grayR);
            cv::waitKey(0);
        }
        vsn::show_loop();
        //----cv::destroyAllWindows();

        cv::Mat mtxL,distL,R_L,T_L;
        cv::Mat mtxR,distR,R_R,T_R;
        cv::Mat Rot, Trns, Emat, Fmat;
        cv::Mat new_mtxL, new_mtxR;

        // Calibrating left camera
        cv::calibrateCamera(objpoints,
                            imgpointsL,
                            grayL.size(),
                            mtxL,
                            distL,
                            R_L,
                            T_L);

        new_mtxL = cv::getOptimalNewCameraMatrix(mtxL,
                                    distL,
                                    grayL.size(),
                                    1,
                                    grayL.size(),
                                    0);

        // Calibrating right camera
        cv::calibrateCamera(objpoints,
                            imgpointsR,
                            grayR.size(),
                            mtxR,
                            distR,
                            R_R,
                            T_R);

        new_mtxR = cv::getOptimalNewCameraMatrix(mtxR,
                                    distR,
                                    grayR.size(),
                                    1,
                                    grayR.size(),
                                    0);        
        return true;
    }
}
//----
CmdCalib::CmdCalib():
    Cmd("Calib commands")
{
    //---- 'omni_stereo'
    {
        string sH = "Omnidir Stereo Calibration \n";
        sH += "   Usage: omni_stereo \n";
        add("omni_stereo", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_omni_stereo(args); }));
    }
    //---- 'stereo'
    {
        string sH = "Stereo Calibration \n";
        sH += "   Usage: stereo dir=<DIR>\n";
        add("stereo", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_stereo(args); }));
    }
}

//------
bool CmdCalib::run_omni_stereo(CStrs& args)
{
    log_i("Calib omni stereo...");
    std::vector<cv::Mat> objectPoints, imagePoints1, imagePoints2;
    cv::Size imgSize1, imgSize2;
   
   
    //---- find pattern
    // TODO: find pattern and points
   //----
    cv::Mat K1, K2, xi1, xi2, D1, D2, idx;
    int flags = 0;
    cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
    std::vector<cv::Mat> rvecsL, tvecsL;
    cv::Mat rvec, tvec;
    double rms = cv::omnidir::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, imgSize1, imgSize2, K1, xi1, D1, K2, xi2, D2, rvec, tvec, rvecsL, tvecsL, flags, critia, idx);
   //-----
    stringstream s;
    vector<cv::Mat> pnts0{objectPoints[0], imagePoints1[0], imagePoints2[0]};
    s << "objPnts[0], imagePoints1[0], imagePoints2[0]:";
    for(auto& p : pnts0)
        s <<"(" << p.rows << "x" << p.cols << "), ";
    //---
    s << "---- calib result ----" << endl;
    s << "K1:" << endl << K1 << endl;
    s << "K2:" << endl << K2 << endl;
    s << "D1:" << endl << D1 << endl;
    s << "D2:" << endl << D2 << endl;
    s << "xi1:" << endl << xi1 << endl;
    s << "xi2:" << endl << xi2 << endl;
    s << "rvec:" << endl << rvec << endl;
    s << "tvec:" << endl << tvec << endl;
    s << "   rms=" << rms << endl;
    log_i(s.str());
    log_e("Under construction...");
    return false;
}


//------
bool CmdCalib::run_stereo(CStrs& args)
{
    log_i("Calib stereo...");
    StrTbl kv;   parseKV(args, kv);
    string sDir = lookup(kv, string("dir"));
    stereo_calib_step1(sDir);
          
        
    return true;
}
