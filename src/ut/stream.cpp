/*
   Author: Sherman Chen
   Create Time: 2023-01-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "ut/cutil.h"

#include <unistd.h>
#include <stdio.h>

#include <termios.h>


using namespace ut;
//----
// return : -1 err , 0 no data or interrupt
//         1 got data
//         2 got data and newline, done
static int c_readln(int fd, string& sln)
{
    sln.clear();

    ssize_t numRead;                    /* # of bytes fetched by last read() */
    size_t totRead;                     /* Total bytes read so far */
    char ch;
    totRead = 0;
    for (;;) {
        numRead = ::read(fd, &ch, 1);

        if (numRead == -1) 
        {
            // normal interrupt
            if (errno == EINTR)         /* Interrupted --> restart read() */
                //continue;
                return 0; 
            else // error happen
                return -1;              /* Some other error */
        } 
        else if (numRead == 0) 
        {      /* EOF */
            if (totRead == 0)           /* No bytes read; return 0 */
                return 0; //  ( no data)
            else                        /* Some bytes read; add '\0' */
                // got some data, reach end of buffer
                return 1; // no data in buffer
        } 
        else // got ch
        {
            /* 'numRead' must be 1 if we get here */
        //    if (totRead < n - 1) {      /* Discard > (n - 1) bytes */
        //        totRead++;
        //    }
            
            sln += ch;
            totRead++;
            if (ch == '\n' || ch=='\r' )
                break; // good, done
        }
    }

    return 2;
}

//-------
bool CStream::readln(string& sln)
{
    sln.clear();

    const double& T  = cfg_.timeout;
    const double& dt = cfg_.t_query_int;
    double t=0;

    while(1)
    {
        int r = c_readln(fd_, sln);
        if(r==2) 
            break; // got newline, done
        else if(r<0) 
            return false; // error found

        else if(r==1) // got some data
        {
            t = 0; // clear timeout
            continue;
        }
        // r==0, no data in buf,
        //   counting timeout
        sys::sleep(dt);
        t += dt;
        if(t > T)
        {
            st_.eTimeout = true;
            return false; // timeout
        }
    }
    return true;
}

//------
bool CStream::readFrm(const string& sHeader, 
                      Buf& buf)
{
    assert(buf.n!=0);

    //---- sync header
    string sr;
    int i=0;
    int L = sHeader.length();
    while(i<buf.n)
    {
        char c;
        int n = ::read(fd_, &c, 1);
        // check err
        bool err = (n==-1)&&(errno != EINTR);
        if(err)
            return false;
        else if( n!=1) // no data
        {
            sys::sleep(cfg_.t_query_int);
            continue;
        }
       
        //---- got char
        buf.p[i] = c;
        if((i<L)&&(sHeader[i]!=c)) // still in sync with header
            i=0;  // reset if not header
        else 
            i++;
    }
    
    return true;

}

//----
void CStream::flush()
{
    tcflush(fd_, TCIFLUSH);
}

