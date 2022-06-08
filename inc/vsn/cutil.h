/*
   Author: Sherman Chen
   Create Time: 2022-05-12
   Email: schen@simviu.com
 */

#pragma once

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <memory>

#include <vector>
#include <map>
#include <list>
#include <queue>

#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace ut
{
    
    using namespace std;

    typedef const string CStr;
    //-----------------------------
    //	Aliase for std::shared_ptr
    //-----------------------------
    //---- aliase shared_ptr<T> to Sp<T>
    template<class T> using Sp = std::shared_ptr<T>;
    
    //---- aliase make_shared<T> to makeSp<T>
    template<class T, class ..._Args>
    inline static std::shared_ptr<T> mkSp(_Args&& ...__args)
    { return std::make_shared<T>(__args...); };
    //---- 
    namespace log{
        extern bool openFile(CStr& sFile);
        extern void dbg(CStr& s);
        extern void inf(CStr& s);
        extern void err(CStr& s);
        extern void errf(CStr& s);
    }
    // shortcuts
    const auto log_i = log::inf;
    const auto log_d = log::dbg;
    const auto log_e = log::err;
    const auto log_ef = log::errf;
    //----
    namespace sys{
        extern string pwd();
        inline void sleepMS(int ms){
            this_thread::sleep_for(chrono::milliseconds(ms) );
        }

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
    struct Px{int x=0;int y=0;};
    struct Color{unsigned char r=0;
                 unsigned char g=0; 
                 unsigned char b=0;
                 unsigned char a=255;};
    //-------------
    // util stuct
    //-------------
    template<typename T=double>
    struct Rng{
        Rng(){};
        Rng(const T& d0, const T& d1):d0(d0),d1(d1){}
        T d0=0.0; 
        T d1=1.0; 
        bool isIn(const T& d)const
        { return (d>=d0)&&(d<=d1); }
    };
    //-------------
    // file utils
    //-------------
    bool fexist(CStr& sf);
}