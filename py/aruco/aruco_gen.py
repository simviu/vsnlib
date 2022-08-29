import cv2 as cv
import numpy as np
import os
from PIL import Image, ImageOps

K_pathw = "4x4_100/"
K_pathwb = "4x4_100_b/"
K_dict = cv.aruco.DICT_4X4_100
WM = 240 # marker width
BD = 40  # boader size
#---------
# gen_marker
#---------
def gen_marker(idx):
    # Load the predefined dictionary
    dictionary = cv.aruco.Dictionary_get(K_dict)

    # Generate the marker
    im = np.zeros((WM, WM), dtype=np.uint8)
    im = cv.aruco.drawMarker(dictionary, idx, WM, im, 1);

    #--- mkdir
    if not os.path.exists(K_pathw):
        os.mkdir(K_pathw)
    if not os.path.exists(K_pathwb):
        os.mkdir(K_pathwb)
    #---- aruco marker origin
    sfw = os.path.join(K_pathw, str(idx)+".png")
    print("save:"+sfw)
    cv.imwrite(sfw, im);
    #---- img add boarder
    imw = Image.open(sfw)
    bimg = ImageOps.expand(imw, border=BD, fill="white")
    sfwb = os.path.join(K_pathwb, str(idx)+".png")
    print("save:"+sfwb)
    bimg.save(sfwb)

#----------
# main
#----------


if __name__ == "__main__":
    for i in range(1,20):
        gen_marker(i)
    
    
