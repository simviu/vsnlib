import numpy as np
import os
from PIL import Image, ImageOps, ImageDraw


K_pathr = "arucod/4x4_100_b/"
K_pathw = "arucod/bars/"
#-----
def gen_name(idx0, N):
    s = K_pathw + "markerbar_" +str(idx0)+ "to" + str(idx0+N-1) + ".png"
    return s;
    
#---------
# gen_bar
#---------
def gen_bar(idx0, N):
    sf0 = os.path.join(K_pathr, str(idx0)+".png")
    im0 = Image.open(sf0)
    w,h = im0.size[0], im0.size[1]
    
    print("gen_bar(): idx0="+str(idx0) + ", N="+str(N))
    print("  marker size: "+str(w)+"x"+str(h))
    print("  Output img size: "+str(w*N)+"x"+str(h))
    
    imw = Image.new('RGB', (w*N, h))
    imw.paste(im0, (0, 0))
    
    for i in range(0, N):
        idx = idx0 + i
        sf = os.path.join(K_pathr, str(idx)+".png")
        x = w*i
        print("Insert :"+sf + " at "+str(x))
        im = Image.open(sf)
        imw.paste(im, (x,0))
        
    #---- done    
    if not os.path.exists(K_pathw):
        os.mkdir(K_pathw)
        
    sfw = gen_name(idx0, N)
    print("save:"+sfw)
    imw.save(sfw)

#----------
# main
#----------


if __name__ == "__main__":
    gen_bar(14, 4)

    
    
