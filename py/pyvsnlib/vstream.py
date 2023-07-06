
import tkinter as tk
import time
from tkinter import ttk
import shlex, subprocess
from threading import Thread
import socket


BORDER_W = 2
TEST_HOST = "127.0.0.1"
TEST_PORT = 2468
T_MIN_FRM_DELAY = 0.01
LN_MAX_CHARS = 1024

#----
class VStreamClient(object):
    def __init__(self, topFrm, sTitle):
        frm = ttk.Frame(topFrm, padding=(3,3,12,12))
       
        #---- title
        lt = tk.Label(frm, text = sTitle)
        lt.grid(row=0, column=0, sticky="news")
        #----
        #---- image
        li = tk.Label(frm, text = sTitle)
        li.grid(row=1, column=0, sticky="news")
        self.l_img_ = li

        #----
        #---- status bar
        ls = tk.Label(frm, text = "status")
        ls.grid(row=2, column=0, sticky="news")
        self.l_status_ = ls
        #----        
        frm.rowconfigure(0, weight=1)
        frm.rowconfigure(1, weight=3)
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
        time.sleep(T_MIN_FRM_DELAY)
        if self.sock_ is None: 
            return 
        
        s = self.recvLn_()
        if s == "" :
            return

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
        buf = self.sock_.recv(l)

        return
        

            

    
#------------------
class TestApp:
    def __init__(self, root):

        lt = tk.Label(root, text = "Widget Test",
                      font = ("Times New Roman", 25))
        lt.grid(row=0, column=0, sticky="news")

        #sCmd = "ping www.yahoo.com"
        #sCmd = "ping"
        sCmd = "./tmp.sh"
        pnl = ConsolePanel(root, "Command Console", sCmd)
        pnl.frm.grid(row=1, column=0, sticky="news")

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
