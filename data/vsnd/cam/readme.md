
## Kitti Camera

  kitti_gray.yml : Kitti Gray Camera calibration (OpenCV)
  kitti_gray_stereo.json: Congfiguration of stereo.

  Baseline:
    Kitti calib.txt, line of P1:
         "P1: 7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 -3.875744000000e+02"
    which is :
        [fx 0 cx fx*b] = [721.5377 0 609.5593 -387.5744 ]

        => baseline = 387.5744 / 721.5377 = 0.537 ,
        
