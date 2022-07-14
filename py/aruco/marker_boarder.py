from PIL import Image, ImageOps
WDIR="output/"
BD = 20

#----- add boarder
def add_border(sf, border):
    img = Image.open(sf)
    bimg = ImageOps.expand(img, border=border)
   
    bimg.save(WDIR + sf)

if __name__ == '__main__':
    
    for i in range(1,10):
        sf = str(i) + ".png"
        add_border(sf, BD)