import cv2 as cv
import numpy as np
import os

K_pathw = "4x4_100/"
K_dict = cv.aruco.DICT_4X4_100

#---------
# gen_marker
#---------
def gen_marker(idx):
    # Load the predefined dictionary
    dictionary = cv.aruco.Dictionary_get(K_dict)

    # Generate the marker
    markerImage = np.zeros((200, 200), dtype=np.uint8)
    markerImage = cv.aruco.drawMarker(dictionary, idx, 200, markerImage, 1);
    pathw = K_pathw
    if not os.path.exists(pathw):
        os.mkdir(pathw)

    sfw = os.path.join(pathw, str(idx)+".png")
    print("sfw:"+sfw)
    cv.imwrite(sfw, markerImage);

#----------
# main
#----------


if __name__ == "__main__":
    for i in range(1,10):
        gen_marker(i)
    
    
