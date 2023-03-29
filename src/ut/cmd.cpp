
/*
   Author: Sherman Chen
   Create Time: 2023-01-17
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "ut/cutil.h"



using namespace ut;

namespace
{
    // internal function
    class CmdFunc : public Cmd{
    public:
        CmdFunc()
        {
           
            add(":halt", "(system halt loop)",
            [&](CStrs& args)->bool{  
                while(1) sys::sleep(0.01);
                return true;
            });

            //----
            add(":sleep", "t=<SEC>",
            [&](CStrs& args)->bool{  
                double t=0;
                if(!KeyVals(args).get("t", t))
                    return false;
                sys::sleep(t);
                return true;
            });
        }
    };  CmdFunc cmdFunc_;

}


//--------------------
// Cmd
//--------------------
string Cmd::rm_comment(CStr & s)const
{
    auto n = s.find('#');
    return s.substr(0, n);
}
//---- 
string Cmd::usage()const
{
    string s;
    s += "  Command line options:\n";
    s += "    --help  : Help\n";
    s += "    --console  : Console\n";
    s += "    --file <CMD_FILE> : Run cmd file\n";
    s += "    --server port=<PORT> : TCP server\n";
    s += "    <CMD> [ARGS] \n";
    return s;
}
//----
bool Cmd::run_func(CStrs& args)
{
    assert(args.size()!=0);
    string scmd=args[0];
    assert(scmd!="");
    KeyVals kvs(args);
    return cmdFunc_.run(args);
}

//----
bool Cmd::runln(const string& sLn_in)
{
    string sLn = ut::remove(sLn_in, '\n');
    auto sLns = tokens(sLn, ';');
    for(auto& s : sLns)
    {
        if(s=="")continue;
        auto args = tokens(s, ' ');
        if(args.size()==0)
            continue;
        //---- check embedded functions
        string scmd = args[0];
        if(scmd=="") continue;
        bool ok = true;
        if(scmd[0]==':')
            ok = run_func(args);
        //---- run this cmd
        else ok = run(args);
        //--- exit loop
        if(!ok) return false;
    }
    return true;
}

//--------
bool Cmd::run(int argc, char ** argv)
{
    bool ok = true;
    try{
        ok = run_core(argc, argv);
    }
    catch(const std::exception& e)
    {
        stringstream s;
        s << "Cmd::run() exception :'";
        s << e.what() << "'";
        log_e(s.str());
        ok = false;
    }
    return ok;
}

//--------
bool Cmd::run_core(int argc, char ** argv)
{

    //---- no arg
    if(argc==1)
    {
        log_i(usage());
        return true;
    }

    //---- run with args
    Strs args;
    for(int i=1;i<argc;i++)
        args.push_back(argv[i]);
    
    //--- check script -f
    string scmd = args[0];
    if(scmd=="--help")
    {
        log_i(help());
        return true;
    }
    else if(scmd=="--file")
    {
        if(args.size()<2)
        {
            log_e("Expect <FILE> after '-f'");
            return false;
        }
        //----
        return runFile(args[1]);
    }
    else if(scmd=="--server")
        return run_server(args);
    else if(scmd=="--console")
        return run_console();
    else if(args.size()==1)// TODO: ?
        return runln(args[0]);
    else 
        return run(args);
    return true;
}
//----
bool Cmd::runFile(CStr& sf)
{
    log_i("Cmd::runFile() :'"+sf+"'...");
    ifstream f;
    f.open(sf);
    if(!f.is_open())
    {
        log_ef(sf);
        return false;
    }
    //
    int i=0;
    bool ok = true;
    while(!f.eof())
    {
        i++;
        string si;
        getline(f, si);
        string si2 = rm_comment(si);
        string s = ut::remove(si2, '\n');
        if(s=="")continue;
        log_i("Run cmd (line "+to_string(i)+"):'"+s+"'");
        ok = runln(s);
        if(!ok)
        {
            log_e("Line "+to_string(i)+" in '"+sf+"':");
            log_e("  Cmd fail:'"+s+"'");
            break;
        }
    }
    //----
    if(ok)
        log_i("Cmd::runFile() done OK");
    else
        log_e("Cmd::runFile() stopped with Error");
    return ok;

}
//---- Cmd server ack
string Cmd::Ack::str()const 
{
    stringstream s;
    s << "run_ok=" << run_ok << endl;
    s << "log:\n";
    s << s_log << endl;
    return s.str();
}
//---
string Cmd::Ack::enc()const
{
    //---- A simple protocol wrap output
    // into multiple lines:
    //  'cmd_ack\n' 
    //  'cmd_ok=[true|false]\n'
    //  '[LOG_LINE\n | ...]'
    //  'cmd_ack_end\n'
    string s;
    s += "cmd_ack\n";
    string s_ok = "cmd_ok=";
    s_ok += run_ok?"true":"false" ;
    s_ok += "\n";

    s += s_ok + s_log + "\n"; 
    s += "cmd_ack_end\n";
    return s;
}
//----
bool Cmd::Ack::dec(CStrs& ss)
{
 //   auto ss = tokens(sLns, '\n');
    if(ss.size()<3)
    {
        log_e("Cmd::Ack::dec() expect at least 3 lines");
        return false;
    }
    //----
    if( (ss[0]!="cmd_ack") ||
        (ss[ss.size()-1]!="cmd_ack_end") )
    {
        log_e("Cmd::Ack::dec() expect 'cmd_ack' and 'cmd_ack_end'");
        return false;        
    }
    //----
    KeyVals kvs(ss);
    //----
    string s_ok = kvs["cmd_ok"];
    if(s_ok=="true") run_ok = true;
    else if(s_ok=="false") run_ok = false;
    else {
        log_e("Cmd::Ack::dec() expect 'true' or 'false'");
        return false;
    } 
    //----
    s_log = "";
    for(int i=2;i<ss.size();i++)
        s_log += ss[i];
    return true;
}


//----- arm server
bool Cmd::run_server(CStrs& args)
{

    socket::Server svr;

    StrTbl kv; parseKV(args, kv);
    string s_port = lookup(kv, string("port"));
    int port=0; 
    if(!s2d(s_port, port))
    {
        log_e(" failed to get para 'port'");
        return false;
    }
   
    //-----
    bool ok = svr.start(port);
    if(!ok)
    {
        log_e("Failed to start server at port:"+to_string(port));
        return false;
    }
    //---- set log call back (TODO: option)
    string s_log;
    utlog::setCallbk([&](CStr& s){
        s_log += s;
    });
    
    //---- server started
    while(svr.isRunning())
    {
        string slnr;
        if(!svr.isConnected())
        {
            sys::sleep(0.2);
            continue;
        } 

        //----
        log_i("Cmd server wait...");
        if(!svr.recvLn(slnr)) 
        {
            sys::sleep(0.2);
            continue;
        }
        //-----
        string sln = ut::remove(slnr, '\n');
        if(sln=="")continue;
        //---
        //---- run cmd
        log_i("Cmd server recv and run:'"+sln+"'");

        //---- run session
        s_log = ""; // clear log
        Ack ack;
        ack.run_ok = this->runln(sln);
        ack.s_log = s_log;
        string sr = ack.enc();
        
        //---
        log_d("  send ack...");
        svr.send(sr);
        sys::sleep(0.2);
        log_d("done\n");
    }
    log_i("Server shutdown");
    //---- 
    return true;
}

//----
bool Cmd::run_console()
{
    log_i("Cmd console, 'help' for help, 'quit' to exit.\n");
    while(1)
    {
        log_s("> ") ;
        string sln;
        std::getline(std::cin, sln);
        
        //--- check quit
        if(sln=="quit") break;

        
        //--- run
        runln(sln);
    }
    return true;
}
//-----
bool Cmd::run(CStrs& args)
{
    //--- run actual cmd
    if(f_!=nullptr)
    {
        if(f_(args)) return true;
        log_e("  Cmd failed, usage: '"+sHelp_ +"'");
        return false;
    }

    // check subcmds
    if(cmds_.size()==0) 
    {
        log_e("Cmd not init and no subcmds");
        return false;
    }
    //-------
    string sc = args[0];
    if(sc=="help")
    {
        log_i(help());
        return true;

    }
    
    //----
    auto p = lookup(cmds_, sc);
    if(p==nullptr)
    {
        log_e("can't find subcmd:'"+sc+"'");
        return false;
        
    }
    //---- run subcmd
    Strs ss = args;
    ss.erase(ss.begin());
    return p->run(ss);

}
//-----
string Cmd::help(const string& s_prefix)const
{
    string s= s_prefix + " "+ sHelp_ + "\n";
    for(auto& it : cmds_)
    {
        auto& sCmd = it.first;
        auto p = it.second;
        s += p->help("  "+s_prefix + " " + sCmd);        
    }
   
    return s;
}




