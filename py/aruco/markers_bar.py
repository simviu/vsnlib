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
    
    #--- draw ref at bottom/left corner
    drawer = ImageDraw.Draw(imw)
    ln = [( 0 , h*0.92 ), (w*0.082 , h )] 
#    drawer.line(ln, fill=(0, 0, 0), width=3) 
    drawer.ellipse((-h*0.025, h*0.975, w*0.025, h*1.025), fill=(0, 0, 0))

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
    gen_bar(10, 4)
    gen_bar(14, 4)
    gen_bar(20, 4)
    gen_bar(24, 4)
    gen_bar(30, 4)
    gen_bar(34, 4)

    
    
