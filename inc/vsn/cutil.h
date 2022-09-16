/*
   Author: Sherman Chen
   Create Time: 2022-05-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <memory>
#include <cmath>

#include <vector>
#include <map>
#include <set>
#include <list>
#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

namespace ut
{
    
    using namespace std;

    //------------
    // string utils
    //------------
    using CStr = const string;
    using Strs = vector<string>;
    using CStrs = const vector<string>;
    using StrTbl = map<string, string>;
    using CStrTbl = const StrTbl;
    //--- 
    inline string lookup(CStrTbl& m, CStr& sk)
    {  auto it=m.find(sk); 
        if(it==m.end()) return ""; return it->second; }

    //--- parse key/value table, e.g.: 
    // file=a.txt n=10 ...
    extern bool parseKV(CStrs& ss, StrTbl& kv);
    inline bool has(CStrTbl& m, CStr& sk)
    { auto it=m.find(sk); return it!=m.end(); }
    //-----------------------------
    //	Aliase for std::shared_ptr
    //-----------------------------
    //---- aliase shared_ptr<T> to Sp<T>
    template<class T> using Sp = std::shared_ptr<T>;
    
    //---- aliase make_shared<T> to makeSp<T>
    template<class T, class ..._Args>
    inline static std::shared_ptr<T> mkSp(_Args&& ...__args)
    { return std::make_shared<T>(__args...); };

    //---- namespace fn
    namespace fn
    {
        extern string nopath(const string& s);
    }
    //-----------
    // container utils
    //-----------
    template<class T>
    inline Sp<T> lookup(map<string, Sp<T>>& m, CStr& s)
    { auto it = m.find(s); 
        return(it==m.end())?nullptr:it->second; }
    

    //-----------
    // math
    //-----------
    inline double toRad(double d){ return d*M_PI/180.0;  };
    inline double toDgr(double d){ return d*180.0/M_PI;  };
    //-----------
    // log
    //-----------
    namespace utlog{
        extern bool openFile(CStr& sFile);
        extern void dbg(CStr& s);
        extern void inf(CStr& s);
        extern void err(CStr& s);
        extern void errf(CStr& s);
    }
    // shortcuts
    const auto log_i = utlog::inf;
    const auto log_d = utlog::dbg;
    const auto log_e = utlog::err;
    const auto log_ef = utlog::errf;
    //----
    namespace sys{
        extern string pwd();
        inline void sleepMS(int ms){
            this_thread::sleep_for(chrono::milliseconds(ms) );
        }
        //--- time functions
        using Time = std::chrono::high_resolution_clock::time_point;
        inline Time now()
        { return std::chrono::high_resolution_clock::now(); }
        inline double elapse(const Time t1, const Time t2)
        { std::chrono::duration<double> e=t2-t1; return e.count(); }
        //----
        class FPS{
        public:
            struct Cfg{ int N_avg=10; };
            Cfg cfg_;
            // check positive as valid
            void tick();
            double fps()const{ return fps_; }
        protected:
            double fps_=-1;
            Time t_ = now();
            list<double> dts;
        };
    }
    
    //----------------
    // multiple thread
    //----------------
    namespace mth{
        //-----------
        // Pipe
        //-----------
        // Cross thread data delivery
        //   with mutex / conditiona var
        template<class T>
        struct Pipe{
            void push(T d){
                std::unique_lock<std::mutex> ul(m_);  
                que_.push(d);
                cv_.notify_one(); 
            }
            T wait()
            {
                std::unique_lock<std::mutex> ul(m_);  
                cv_.wait(ul,[&] {return que_.size()!=0;});  
                auto p = que_.front();
                que_.pop();
                return p;
            }
            void clear(){
                std::unique_lock<std::mutex> ul(m_);  
                while(!que_.empty())
                    que_.pop();
            }
        protected:
            queue<T> que_;
            std::condition_variable cv_;
            std::mutex m_;
        };   
    }
    //----------
    // draw data
    //----------
    struct Px{
        Px(){}
        Px(int x, int y):x(x),y(y){}
        int x=0;int y=0;
        Px operator - (const Px& d)const
        { return {x - d.x, y - d.y}; }
        Px operator + (const Px& d)const
        { return {x + d.x, y + d.y}; }
        void operator -= (const Px& d)
        {  x -= d.x; y -= d.y; }
        void operator += (const Px& d)
        {  x += d.x; y += d.y; }
        string str()const 
        { stringstream s; 
          s << x <<","<< y; return s.str(); } 
    };
    struct Color{
        uint8_t r=0;
        uint8_t g=0; 
        uint8_t b=0;
        uint8_t a=255;
        string str()const 
        { stringstream s; 
          s << (int)r <<","<< (int)g << ","
             << (int)b << "," << (int)a; 
          return s.str(); } 
    };
    struct Sz{
        Sz(){}
        Sz(int w, int h):w(w), h(h){}
        int w=0;
        int h=0;
        bool isIn(const Px& px)
        { return (px.x>=0) && (px.y>=0) && (px.x<w) && (px.y<h); }
        string str()const
        {  stringstream s; s << w << "," << h << endl; return s.str(); }
    };
    inline ostream& operator << (ostream& s, const Sz& sz)
    {  s << sz.w << ", " << sz.h; return s; }
    //-----    
    struct Rect{
        Rect(){}
        Rect(const Px& c, const Sz& sz):cntr(c), sz(sz){}
        Px cntr;
        Sz sz;
        Px p0()const { return Px(cntr.x - sz.w*0.5, cntr.y - sz.h*0.5); }
        Px p1()const { return Px(cntr.x + sz.w*0.5, cntr.y + sz.h*0.5); }
    };
    //-------------
    // util stuct
    //-------------
    template<typename T=double>
    struct Rng{
        Rng(){};
        Rng(const T& d0, const T& d1):d0(d0),d1(d1){}

        bool isIn(const T& d)const
        { return (d>=d0)&&(d<=d1); }
        void upd(const T& d)
        {   if(!val()) d0=d1=d; 
            else if(d>d1)d1=d; else if(d<d0)d0=d; }
        T len()const{ return fabs(d1-d0); }
        T mid()const{ return (d0+d1)*0.5; }
        bool val()const{ return d1>=d0; }
        string str()const
        { stringstream s; s << d0 <<"," << d1; return s.str(); }
        T d0=1.0; 
        T d1=0; 
    };
    //-------------
    // file utils
    //-------------
    bool fexist(CStr& sf);

    //-------------
    // Test
    //-------------
    class Test{
    public:
        Test(){};
        Test(map<string, Sp<Test>>& ts):tests_(ts){}
        virtual bool run();
        virtual bool run(const string& s);
        void add(const string& s, Sp<Test> p)
        { tests_[s] = p; }
        string getTestsStr()const;

        // sub tests
        map<string, Sp<Test>> tests_;

    };

    //-------------
    // Cmds
    //-------------
    // string cmd line handler
    class Cmd{
    public:
        using Fun=function<bool(CStrs& args)>;
        Cmd(){}
        Cmd(CStr& sHelp):sHelp_(sHelp){}
        Cmd(CStr& sHelp, Fun f):sHelp_(sHelp), f_(f){}
        void add(CStr& s, Sp<Cmd> p)
        { cmds_[s]=p; }
        virtual bool run(CStrs& args);
        bool parse(CStr& s);
        bool runFile(CStr& sf);
    protected:
        string sHelp_;
        Fun f_=nullptr;
        map<string, Sp<Cmd>> cmds_;
        
    };
}