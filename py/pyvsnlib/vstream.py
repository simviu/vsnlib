
import tkinter as tk
import time
from tkinter import ttk
import shlex, subprocess
from threading import Thread
import socket
import cv2
import numpy as np
from PIL import Image, ImageTk



BORDER_W = 2
TEST_HOST = "127.0.0.1"
TEST_PORT = 2468
T_MIN_FRM_DELAY = 0.01
LN_MAX_CHARS = 1024

#----
class VStreamClient(object):
    def __init__(self, topFrm, sTitle, scale=(1.0,1.0)):
        self.cfg = {}
        self.cfg['scale'] = scale
        frm = ttk.Frame(topFrm, padding=(3,3,12,12))
       
        #---- title at(0,0)
        lt = tk.Label(frm, text = sTitle)
        lt.grid(row=0, column=0, sticky="news")
        #----
        #---- image at(1,0)
        li = tk.Label(frm, text = sTitle)
        li.grid(row=1, column=0, sticky="news")
        self.l_img_ = li
        frm.columnconfigure(0, minsize=640)
        frm.rowconfigure(1, minsize=480)

        #----
        #---- status bar at(2,0)
        ls = tk.Label(frm, text = "status")
        ls.grid(row=2, column=0, sticky="news")

        self.l_status_ = ls
        #----        
        frm.rowconfigure(0, weight=1)
        frm.rowconfigure(1, weight=10)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)

        self.frm  = frm

        return
    
    #----
    def connect(self, sHost, port):
        #------
        print("ArmTcp connect to '"+sHost+"'" + str(port)+"...")
        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.sock_ is None:
            raise Exception("socket failed")
        
        self.sock_.connect((sHost, port))
        self.sock_.setblocking(1)
        # TODO: check connection
        print("connected")


        #----
        thd = Thread(target=self.run_bk_func_,  daemon=True)
        thd.setDaemon(True)
        thd.start()
        self.run_thd_ = thd
        return


    #------------- private -------------
    def recvLn_(self):
        if self.sock_ is None:
            raise Exception('Not connected')
            
        s = ""
        for i in range(LN_MAX_CHARS):
            b = self.sock_.recv(1)
            c = b.decode('UTF-8')
            if c == "\n":
                return s
            #print("recv:"+c)
            s = s + c

        return s    
    
    #-----
    def run_bk_func_(self):
        if self.sock_ is None: 
            return 

        while True:
            time.sleep(T_MIN_FRM_DELAY)
            self.recv_frm_()
        

        return

    #---- recv buf
    def recv_buf_(self, N):
        sock = self.sock_
        chunks = []
        bytes_recd = 0
        while bytes_recd < N:
            chunk = sock.recv(min(N - bytes_recd, 2048))
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return b''.join(chunks)

    #--- 
    def recv_frm_(self):
        print("[dbg]: recvLn_()...")
        s = self.recvLn_()
        if s == "" :
            return
        print("[dbg]: recv s='"+s+"'")

        #---- prepare err msg
        sExp = "'dtype=vstream.image buf_len=<N>\n'"
        se = "vstream header incorrect,"
        se = se + " expect '"+sExp+"'"
        sErr = se + " recv: '"+s +"'"

        #---- header is one line 
        #   s << "dtype=vstream.image ";
        #   s << "buf_len=" << n << "\n";
        ss = s.split()

 
        #---
        if len(ss)<2 or ss[0] != "dtype=vstream.image": 
            raise(sErr)
        #---
        sh2,sl = ss[1].split('=')
        if sh2 != "buf_len" or sl is None:
            raise(sErr)

        l = int(sl)
        print("[dbg]: buf len:"+str(l))
        #dt = self.sock_.recv(l)
        dt = self.recv_buf_(l)
        print("[dbg]: recv dt, len="+str(len(dt)))
        #---


        #---
        buf = np.asarray(bytearray(dt), dtype="uint8")
        im = cv2.imdecode(buf,cv2.IMREAD_COLOR)
        print("[dbg]:im dec done")

        #cv2.imshow("image",im)
        #cv2.waitKey(10)
        #----
        scl = self.cfg['scale']
        w,h = im.shape[1], im.shape[0]
        w,h = int(w * scl[0]), int(h * scl[1])
        im  = cv2.resize(im, (w,h))
        #----
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(im)
        imgtk = ImageTk.PhotoImage(image=img)
        #---- dbg
        #imgtk = ImageTk.PhotoImage(Image.open("./t1.jpg"))
        lb = self.l_img_
        lb.configure(image=imgtk)
        lb.image = imgtk
        return
        

            

    
#------------------
class TestApp:
    def __init__(self, root):

        lt = tk.Label(root, text = "Widget Test",
                      font = ("Times New Roman", 25))
        lt.grid(row=0, column=0, sticky="news")

        pnl = VStreamClient(root, "vstream test", (0.5,0.5))
        pnl.frm.grid(row=1, column=0, sticky="news")
        pnl.connect(TEST_HOST, TEST_PORT)

        root.rowconfigure(0, weight=1)
        root.rowconfigure(1, weight=10)
        root.columnconfigure(0, weight=1)

#----------
# main
#----------
if __name__ == "__main__":
    root = tk.Tk()
    app = TestApp(root)
    root.mainloop()
