#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>
#include <cstring>
#include <list>
#include <string>
#include <vector>
#include <iostream>
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/intrusive/parent_from_member.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <boost/exception/errinfo_at_line.hpp>
#include <boost/algorithm/string/find.hpp>
#include <boost/range/iterator_range.hpp>
//#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/program_options.hpp>
#include "singleton.h"
#include "myerror.h"
#include "log.h"
#include "json.h"
#include "dbc.h"

#include <boost/container/flat_map.hpp>
//#include "message.h"

#define NO_HEADER (boost::archive::no_header)

typedef unsigned int UInt;

using boost::asio::ip::tcp;
namespace placeholders = boost::asio::placeholders;
namespace multi_index = boost::multi_index;

namespace posix = boost::asio::posix;
// static char* const arg0_;

namespace error {
    struct fatal : myerror { using myerror::myerror; };
    struct signal : myerror { using myerror::myerror; };
    struct not_found : myerror { using myerror::myerror; };
    struct size : myerror { using myerror::myerror; };
    struct io : myerror { using myerror::myerror; };
    struct output_not_exist : myerror { using myerror::myerror; };

    // typedef boost::error_info<struct I,size_t> info_int;
    // typedef boost::error_info<struct S,std::string> info_str;
    // typedef boost::error_info<struct E,boost::system::error_code> info_ec;
    template <typename T> inline boost::error_info<T,T> info(T const& x) { return boost::error_info<T,T>(x); }
    inline boost::error_info<char const*,char const*> info(char const* x) {
        return boost::error_info<char const*,char const*>(x);
    }

} /// namespace error

//template <typename T>
//inline std::ostream& operator<<(std::ostream& out, std::vector<T> const& v)
//{
//    typedef std::vector<T>::const_iterator iterator;
//    return out << boost::make_iterator_range<iterator>(v);
//}

template <typename T,int N>
struct auto_close_fdvec : boost::container::static_vector<T,N>
                    , boost::noncopyable
{
    ~auto_close_fdvec() {
        BOOST_FOREACH(int fd, *this) {
            if (fd > 0)
                close(fd);
        }
    }

    template <typename R> R operator()(R&& a)
    {
        BOOST_FOREACH(T fd, a)
            this->push_back(fd);
        return boost::move(a);
    }
    T operator()(T fd)
    {
        this->push_back(fd);
        return fd;
    }
};

std::string basename(std::string pf)
{
    auto x = pf.find_last_of('/');
    if (x != std::string::npos) {
        pf.erase(0, x+1);
    }
    return boost::move(pf);
}

struct Argv : std::vector<std::string>
{
    template <typename ...Params>
    Argv(std::string const& p, Params... args) : path_(p)
        { append(args...); }

    Argv(std::string const& p) : path_(p) {
        if (!p.empty())
            append(basename(p));
    }

    int execute(int iofd[2]) const;

    std::string const& path() const { return path_; }
    // bool empty() const { return path_.empty(); }

private:
    void append(std::string const& a)
    {
        push_back(a);
        BOOST_ASSERT(size() < 16);
    }
    template <typename ...Params>
    void append(std::string const& a, Params&... args)
    {
        push_back(a);
        append(args...);
    }

    std::string path_;

    friend std::ostream& operator<<(std::ostream& out, Argv const& av)
    {
        std::vector<std::string> const& cv = av;
        return out << av.path() <<" "<< cv;
    }
};

int Argv::execute(int iofd[2]) const
{
    BOOST_ASSERT(size() > 0 && size() < 16);
    typedef boost::container::static_vector<char*,16> Aps;
    Aps av;

    av.push_back( const_cast<char*>(path_.c_str()) );
    BOOST_FOREACH(std::string const& x, *this) {
        av.push_back( const_cast<char*>(x.c_str()) );
    }
    av.push_back(0);

    int mfd = 100;
    if (iofd[1] != STDIN_FILENO) {
        dup2(iofd[1], STDIN_FILENO);
        mfd = std::max(mfd, iofd[1]);
    }
    if (iofd[0] != STDOUT_FILENO) {
        dup2(iofd[0], STDOUT_FILENO);
        mfd = std::max(mfd, iofd[0]);
    }
    for (int fd=3; fd < mfd; fd++) {
        if (close(fd) == 0)
            mfd = std::max(fd+100, mfd);
    }

    execvp(av[0], &av[1]);
    int errN = errno;
    if (errN == EACCES) {
        // LOG << "exec EACCES" << errN << strerror(errN) << av[0];
        std::swap(av[0], av[1]);
        execv("/bin/sh", &av[0]);
    }

    return errN;
}

//static pid_t Forkex(int iofd[2], Argv const& av)
//{
//    pid_t pid = fork();
//    if (pid < 0) {
//        char* es = strerror(errno);
//        LOG << "fork" << pid << es;
//        BOOST_THROW_EXCEPTION(error::fatal() << error::info(es));
//    }
//
//    LOG << av << iofd[0] << iofd[1]; // << errN;
//
//    if (pid == 0) // child
//    {
//        int errN;
//        auto bname = basename(av.path());
//        if (bname == "cat") {
//            Argv av2 ( av.path(), av[0] ); // av.resize(1);
//            errN = av2.execute(iofd); // errN = exec_av(iofd, v);
//        } else {
//            errN = av.execute(iofd); // errN = exec_av(iofd, v);
//        }
//        kill(getppid(), SIGUSR1);
//        LOG << "exec" << strerror(errN) << av;
//        exit(errN);
//    }
//
//    return pid;
//}
//
//static int open_tty_output() { return open("/proc/self/fd/0", O_NOCTTY, O_WRONLY); } //readlink("/proc/self/fd/0");

boost::array<int,2> Pipe()
{
    boost::array<int,2> v;
    if (pipe(&v[0]) < 0) {
        int errN = errno;
        char* es = strerror(errN);
        BOOST_THROW_EXCEPTION(error::fatal() << error::info(errN) << error::info(es));
    }
    return v;
}

inline boost::asio::mutable_buffer mutable_buffer(uint32_t& n)
{
    return boost::asio::mutable_buffer(static_cast<void*>(&n), sizeof(uint32_t));
}
inline boost::asio::const_buffer const_buffer(uint32_t const& n)
{
    return boost::asio::const_buffer(static_cast<void const*>(&n), sizeof(uint32_t));
}

static void save_pid(std::string const& fp) //(const char* sfmt)
{
    boost::filesystem::ofstream out(fp);
    if (out) {
        out << getpid();
    }
}

static boost::property_tree::ptree make_pt(boost::filesystem::path const &fp)
{
    boost::filesystem::ifstream cfs(fp);
    if (!cfs) {
        BOOST_THROW_EXCEPTION(error::io() << error::info(fp));
    }
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(cfs, pt);
    return boost::move(pt);
}

//#define DECL_TAG(X) struct X{ friend std::ostream& operator<<(std::ostream& out,X const& x) {return out<< #X;} }
//DECL_TAG(Soc); DECL_TAG(Dec); DECL_TAG(Enc); DECL_TAG(App);

struct fifo_stream_descriptor : posix::stream_descriptor, boost::noncopyable
{
    std::string fp_;

    ~fifo_stream_descriptor()
    {
        if (!fp_.empty()) {
            remove(fp_.c_str());
        }
    }

    fifo_stream_descriptor(boost::asio::io_service& io_s, std::string const& fp)
        : posix::stream_descriptor(io_s)
        , fp_(fp)
    {
        if (!fp_.empty()) {
            remove(fp_.c_str());
            if (mkfifo(fp_.c_str(), 0660) < 0) {
                LOG << "mkfifo" << fp_ << strerror(errno);
                BOOST_THROW_EXCEPTION(error::fatal());
            }

            int fd = open(fp_.c_str(), O_RDONLY|O_NDELAY);
            if (fd < 0) {
                LOG << "open" << fp_ << strerror(errno);
                BOOST_THROW_EXCEPTION(error::fatal());
            }
            this->assign(fd);
        }
    }
};

struct Main : boost::asio::io_service, singleton<Main>
{
//"yuexia:UsrPuppetList"
    struct Args
    {
        int argc;
        char* const* argv;

        std::string host_; //("127.0.0.1");
        short unsigned int port_; // = "9999";
        std::string conf_; // = "9999";
        std::string pidfile_; // = "9999";

        Args(int ac, char *const av[]);
    };

    boost::asio::signal_set signals_exit_;
    boost::asio::signal_set signals_;
    tcp::acceptor acceptor_;
    tcp::socket socket_;
    // tcp::endpoint endp_;
    // fifo_stream_descriptor cast_;

    ~Main() {
        //thread_io_service_.stop();
        //thread_.join();
    }
    Main(Args const& args); // (int argc, char *const argv[])

    int run();

private:
    void handle_accept(boost::system::error_code ec);
    void handle_connect(boost::system::error_code ec) {}

private:
    static void timer_fn(boost::system::error_code ec);

    boost::asio::deadline_timer timer_;

private:
    static void thread_myinit(Main& self);
    static void thread_fn();
    static void thread_saving(boost::system::error_code ec);

    boost::thread thread_;
    boost::asio::io_service thread_io_service_;
    boost::asio::deadline_timer thread_timer_;
};

//UInt Main::alloc_msgid()
//{
//    auto reply = redis::command("INCR", "msg/id");
//    if (!reply || reply->type != REDIS_REPLY_INTEGER)
//    {
//        LOG << "redis msg/id error";
//        BOOST_THROW_EXCEPTION(error::fatal());
//    }
//    return reply->integer;
//}

//void Main::thread_saving(boost::system::error_code ec)
//{
//    if (ec) {
//        LOG << ec << ec.message();
//        return;
//    }
//    Main& self = instance();
//    int secs = 10;
//
//    auto reply = redis::command("LRANGE", "mq/body", -500, -1);
//    if (!reply || reply->type != REDIS_REPLY_ARRAY) {
//        LOG << "redis mq/body error";
//        secs = 30;
//
//    } else if (reply->elements > 0) {
//        { // sql, message
//            // AUTO_CPU_TIMER("sql:message");
//            std::vector<std::string> vals;
//            boost::format fmt("(%1%,%2%,%3%,'%4%','%5%')");
//            UInt mx = self.my_max_msgid_;
//            for (size_t i = reply->elements; i > 0; i--) {
//                UInt mid = atoi(reply->element[i-1]->str);
//                if (mid <= mx) {
//                    LOG << mid << "skip" << mx;
//                    continue;
//                }
//                mx = mid;
//                auto* m = self.cache_get(mid);
//                if (!m) {
//                    LOG << mid << "skip not found";
//                    continue;
//                }
//                vals.push_back(str(fmt % m->msgid % m->fr_uid % m->sidN
//                            % sql::escape(json::encode(m->content)) % sql::escape(m->notification)));
//            }
//            if (!vals.empty()) {
//                sql::exec("INSERT INTO message(id,uid,sid,content,notification) VALUES " + boost::join(vals,","));
//            }
//            self.my_max_msgid_ = mx;
//        }
//        redis::command("LTRIM", "mq/body", 0, -int(reply->elements+1));
//        secs = 1;
//        LOG << reply->elements;
//    }
//
//    self.thread_timer_.expires_from_now(boost::posix_time::seconds(secs));
//    self.thread_timer_.async_wait(&Main::thread_saving);
//}

//void Main::thread_myinit(Main& self)
//{
//    self.my_max_msgid_ = 0;
//
//    sql::datas datas("SELECT MAX(id) FROM message");
//    if (sql::datas::row_type row = datas.next()) {
//        self.my_max_msgid_ = std::atoi(row[0]);
//    }
//
//    LOG << self.my_max_msgid_;
//}

//void Main::thread_fn()
//{
//    try
//    {
//        thread_myinit(instance());
//
//        thread_saving(boost::system::error_code());
//        // boost::asio::io_service::work work(thread_io_service_);
//        instance().thread_io_service_.run();
//    } catch (myerror& e) {
//        LOG << e;
//    }
//    LOG << "thread bye.";
//}

//void Main::timer_fn(boost::system::error_code ec)
//{
//    if (ec) {
//        return;
//    }
//    auto& self = instance();
//    time_t tpc = time(0);
//
//    if (!self.empty()) {
//        // auto& sk = self.back();
//        // if (tpc - sk.tpa_ > 90) { LOG << sk << "timeout"; }
//    } else { LOG << "empty"; }
//
//    while (!self.cache_.empty())
//    {
//        BOOST_STATIC_CONSTANT(int, cache_max_duration=60*60*6);
//        time_t tpa = self.cache_.back().tpa_;
//        if (tpc - tpa < cache_max_duration) {
//            break;
//        }
//        self.cache_.pop_back();
//    }
//
//    self.timer_.expires_from_now(boost::posix_time::seconds(15));
//    self.timer_.async_wait(Main::timer_fn);
//}

//message_body const* Main::cache_get(UInt mid, boost::system::error_code* ec) const
//{
//    static message_body tmp_;
//    tmp_.msgid = mid;
//    Main& self = const_cast<Main&>(*this);
//
//    auto p = self.cache_.push_front(tmp_);
//    if (!p.second) {
//        self.cache_.relocate(self.cache_.begin(), p.first);
//        p.first->tpa_ = time(0);
//        return &*p.first;
//    }
//    auto& body = const_cast<message_body&>(*p.first);
//    {
//        using namespace boost;
//
//        format fmt("SELECT uid,UNIX_TIMESTAMP(ctime),sid,content,notification FROM message WHERE id=%1%");
//        sql::datas datas(fmt % body.msgid);
//        sql::datas::row_type row = datas.next();
//        if (!row) {
//            LOG << mid << "not found";
//            if (ec)
//                *ec = make_error_code(boost::system::errc::no_message_available);
//            return 0; //BOOST_THROW_EXCEPTION(error::not_found() << error::info(mid));
//        }
//
//        body.fr_uid = lexical_cast<UInt>(row[0]);
//        body.tp_create = lexical_cast<time_t>(row[1]);
//        body.sidN = lexical_cast<int>(row[2]); // & 0x7f;
//        body.content = json::decode<json::object>( row.at(3) ).value();
//        body.notification = row[4];
//        body.tpa_ = time(0);
//    }
//    return &body;
//    //auto i = self.cache_.emplace(self.cache_.end(), mid, fr_uid, tp_creat, sidN, jo, notification);
//    //return *multi_index::project<1>(self.cache_,i);
//}

//UInt Main::post(destinations & dsts, message_body&& body)
//{
//    BOOST_ASSERT(dsts.msgid==0 && body.msgid==0);
//
//    if (dsts.empty()) {
//        BOOST_THROW_EXCEPTION(error::size());
//    }
//
//    // std::string const& sid = dsts.front().sid;
//    UInt mid = body.msgid = alloc_msgid();
//    dsts.msgid = mid;
//    redis::context redx;
//    {
//        boost::asio::streambuf sbuf; {
//            boost::archive::binary_oarchive oa(sbuf, NO_HEADER);
//            serializ(oa, dsts);
//        }
//        // redx.append("HSET", "msg/cache", mid, sbuf.data());
//        redx.append("LPUSH", "mq/body", sbuf.data());
//    }/*v*/{
//        boost::asio::streambuf sbuf; {
//            boost::archive::binary_oarchive oa(sbuf, NO_HEADER);
//            serializ(oa, dsts);
//        }
//        redx.append("LPUSH", "mq/dests", sbuf.data());
//    }
//
//    // time_t tpc = time(0);
//    // redx.append("ZADD", "msg/cache/a", tp, mid);
//    // redx.append("ZADD", "msg/deadline", tpc+live_seconds(sid), mid);
//
//    redx.reply();
//    this->cache_put( body );
//
//    return mid;
//}

//void async_recv(Iterator<0> it)
//{
//    auto & in = it->inbuf_;
//    mark_live(it);
//    boost::asio::async_read(*it, boost::asio::buffer(mutable_buffer(in.len)), Handle_read_size(it));
//}
//
////template <int Io, typename ...Params> void async_send(Iterator<Io> it, Params const& ...ps);
//template <int Io, typename ...Params>
//void async_send(Iterator<Io> it, Params const& ...ps)
//{
//    auto & out = it->outbufs_;
//
//    auto i = out.emplace(out.end());
//    boost::archive::binary_oarchive oa(*i, NO_HEADER);
//    serializ(oa, ps...);
//
//    {
//        Iterator<1> it1(it);
//        if (it1.count() == 0) {
//            Handle_write wr(it1);
//            wr(boost::system::error_code(), 0);
//        }
//    }
//}

Main::Main(Args const& args) // (int argc, char *const argv[])
    : signals_exit_(*this, SIGINT, SIGTERM, SIGQUIT)
    , signals_(*this, SIGHUP)
    , acceptor_(*this)
    , socket_(*this)
    , timer_(*this)
    , thread_timer_(thread_io_service_)
    // , cast_(*this, args.cast_)
{
    boost::property_tree::ptree conf = make_pt(args.conf_);

    redis::init( conf.get_child("redis") );
    sql::dbc::init( sql::dbc::config(conf.get_child("mysql")) );

    //cache_max_size_ = conf.get<UInt>("cache_max_size", 10000);
    //LOG << cache_max_size_;

    // tcp::resolver resolv(*this); // resolv.resolve(tcp::resolver::query(args.host_, args.port_));
    //tcp::endpoint ep(tcp::v4(), args.port_);
    //acceptor_.open(ep.protocol());
    //acceptor_.set_option(tcp::acceptor::reuse_address(true));
    //acceptor_.bind(ep);
}

int Main::run()
{
    signals_.async_wait(
            [this](boost::system::error_code ec, int sig) {
                LOG << sig;
            });
    signals_exit_.async_wait(
            [this](boost::system::error_code ec, int sig) {
                LOG << sig;
                this->stop();
            });

    if (acceptor_.is_open()) {
        acceptor_.listen();
        acceptor_.async_accept(socket_, boost::bind(&Main::handle_accept, this, placeholders::error));
    // } else { socket_.async_connect(endp_, boost::bind(&Main::handle_connect, this, placeholders::error));
    }

    //timer_fn(boost::system::error_code());
    //thread_ = boost::thread(&Main::thread_fn);

    return boost::asio::io_service::run();
}

Main::Args::Args(int ac, char *const av[])
    : argc(ac), argv(av)
    , conf_("/etc/mcenter.conf")
{
    namespace opt = boost::program_options;

    opt::variables_map vm;
    opt::options_description opt_desc("Options");
    opt_desc.add_options()
        ("help", "show this")
        ("daemon", "daemon")
        ("pid-file", opt::value<std::string>(&pidfile_)->default_value(""), "pid file")
        //("listen,l", "listen mode")
        //("host,h", opt::value<std::string>(&host_)->default_value("0"), "host")
        //("port,p", opt::value<unsigned short>(&port_)->default_value(9999), "port")
        ("conf", opt::value<std::string>(&conf_)->default_value(conf_), "config file")
        ;
    opt::positional_options_description pos_desc;
    pos_desc.add("conf", 1);

    opt::store(opt::command_line_parser(argc, argv).options(opt_desc).positional(pos_desc).run(), vm);

    if (vm.count("help")) {
        std::cout << boost::format("Usage:\n  %1% [Options]\n") % argv[0]
            << opt_desc
            ;
        exit( 0 );
    }

    opt::notify(vm);

    if (vm.count("daemon")) {
        BOOST_ASSERT(!conf_.empty());
        int chdir = (boost::starts_with(conf_,"/") && boost::starts_with(pidfile_,"/"));
        if (daemon(!chdir, 0) != 0) {
            LOG << "daemon" << strerror(errno);
        }
        logging::syslog(LOG_PID|LOG_CONS, 0);
        LOG << conf_ << chdir << pidfile_;
    }

    if (!pidfile_.empty()) {
        save_pid(pidfile_);
    }
}

int main(int argc, char* const argv[])
{
    try
    {
        Main app(Main::Args(argc, argv));
        //Mstart();
        app.run();
    } catch (error::signal& e) {
        LOG << e;
    } catch (myerror& e) {
        LOG << "Exception:" << e.what();
    }

    LOG << "bye.";
    return 0;
}

// struct Handle_write : Iterator<1> 
// {
//     void operator()(boost::system::error_code ec, size_t bytes) const
//     {
//         Iterator<1> const& it = *this;
//         if (ec) {
//             LOG << ec << ec.message();
//             return;
//         }
//         if (it->outbufs_.empty()) {
//             return;
//         }
// 
//         auto & out = it->outbufs_;
// 
//         auto& sbuf = out.front();
//         out.len = htonl(sbuf.size());
// 
//         //boost::array<boost::asio::const_buffer,2> bufs;
//         //bufs[0] = const_buffer(out.len);
//         //bufs[1] = sbuf.data();
//         boost::array<boost::asio::const_buffer,2> bufs{ const_buffer(out.len), sbuf.data() };
// 
//         mark_live(it);
//         boost::asio::async_write(*it, bufs, *this);
//     }
// 
//     Handle_write(Iterator<1> it) : Iterator<1>(it)
//     {}
// };
// 
// void async_recv(Iterator<0> it);
// 
// int logik_entry(Iterator<0> it, boost::archive::binary_iarchive& ar);
// 
// struct Handle_read : Iterator<0>
// {
//     void operator()(boost::system::error_code ec, size_t bytes)
//     {
//         if (ec) {
//             LOG << ec << ec.message();
//             return;
//         }
//         Iterator<0>& it = *this;
//         auto & in = it->inbuf_;
// 
//         in.commit(bytes);
//         {
//             int x;
//             boost::archive::binary_iarchive ia(in, NO_HEADER);
//             if ( (x = logik_entry(it, ia)) < 0) {
//                 LOG << x;
//             } else {
//                 async_recv(*this);
//             }
//         }
//     }
// 
//     Handle_read(Iterator<0> it) : Iterator<0>(it)
//     {}
// };
// 
// struct Handle_read_size : Iterator<0>
// {
//     void operator()(boost::system::error_code ec, size_t bytes)
//     {
//         if (ec) {
//             LOG << ec << ec.message();
//             return;
//         }
// 
//         Iterator<0> & it = *this;
//         auto& in = it->inbuf_;
// 
//         in.len = htonl(in.len);
// 
//         mark_live(it);
//         boost::asio::async_read(*it, in.prepare(in.len), Handle_read(it));
//     }
// 
//     Handle_read_size(Iterator<0> it) : Iterator<0>(it)
//     {}
// };
// 
void Main::handle_accept(boost::system::error_code ec)
{
    if (ec) {
        LOG << ec << ec.message();
        return;
    }

    //Iterator<0> it( emplace(begin(), std::move(socket_)) );
    //async_recv(it);

    socket_.close(ec);
    //socket_.open(tcp::v4());
    acceptor_.async_accept(socket_, boost::bind(&Main::handle_accept, this, placeholders::error));
}


