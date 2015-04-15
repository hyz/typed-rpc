#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
//#include "prettyprint.hpp"
#include "async_message.hpp"
namespace filesystem = boost::filesystem;

namespace Echo {

typedef Message::Pair< Message::Request<>, Message::Response<> > ___reserved___;

using Message::Request;
using Message::Response;

typedef Message::Table<
    ___reserved___
  , Message::Pair< Request<std::string>, Response<std::string> >
  , Message::Pair< Request<int, std::string>, Response<int, std::string> >
  , Message::Pair< Request<std::map<int,std::string>>, Response<std::map<int,std::string>> >
> Message_table;

struct Message_handle : service_def<Message_handle,Message_table> //::: server-side
{
    /// handle all Msg_echo message
    template <typename Reply, typename...T>
    void operator()(Reply reply, Msg_echo, T&& ...t) const
    {
        reply(std::forward<T>(t)...);
    }
    Message_handle(ip::tcp::socket* s) : service_def<Message_handle,Message_table>(s) {}
};

struct Client : agent_def<Client,Message_table>, singleton<Client>
{
    Client(boost::asio::io_service& io_s, ip::address ipa, unsigned short port)
        : agent_def<Client,Message_table>(boost::ref(io_s), ipa, port)
    {}
};

} // Echo // namespace

/// / // / // / /// /// / // / // / ///
#include <iostream>
#include <boost/timer/timer.hpp>

template <typename... T>
static void _ensure(bool y, T&&... t)
{
    if (!y) {
        if (sizeof...(t) > 0) {
            int v[] = { (void(std::cerr<<t<<" "),0)... };
            (void)v;
        }
        std::cerr <<" abort.\n";
        abort();
    }
}
#define ENSURE(...) _ensure(__VA_ARGS__,__LINE__)

static void handle_accept(Acceptor* a, boost::system::error_code ec)
{
    if (!ec) {
        std::cerr << ec <<" "<< ec.message() <<"\n";
        return;
    }
    auto ptr = server<Echo::Message_handle>::construct(&a->socket);
    ptr->start(); //::: start
    a->async_accept([a](boost::system::error_code ec){ handle_accept(a, ec); });
}

int server_main(ip::tcp::endpoint ep1)
{
    boost::asio::io_service io_s;

    server<Echo::Message_handle> srv;
    Acceptor a(io_s, ep1);
    a.async_accept([&a](boost::system::error_code ec){ handle_accept(&a, ec); });

    io_s.run();
    static_cast<void>(srv); // silent warning
    return 0;
}

template <typename Data>
static void echo_test(ip::address host, unsigned short port, int n_req, Data&& data)
{
    boost::asio::io_service io_s;
    Echo::Client cli(io_s, host, port);

    int n_rsp = 0; {
        boost::timer::auto_cpu_timer t;
        for (int i=0; i < n_req; ++i) {
            cli.async_do([&n_rsp,&data](Data const& u) { ENSURE(u==data); ++n_rsp; }
                    , data);
        }
        io_s.run();
    }
    ENSURE(n_req==n_rsp, n_req, n_rsp, data);

    std::cout << n_req
        <<" "<< cli.sockets.size()
        <<"\n";
}

static ip::tcp::endpoint make_endpx(char const* host, char const* port)
{
    if (host)
        return ip::tcp::endpoint(ip::address::from_string(host), std::stoi(port));
    return ip::tcp::endpoint(ip::tcp::v4(), std::stoi(port));
}

///
// start server:
//      bin/example -l <port>
///
// start client:
//      bin/example <host> <port>
int main(int ac, char *const av[])
{
    if (ac < 3) {
        return 3;
    }

    if (av[1] == std::string("-l")) {
        return server_main(make_endpx(0, av[2]));
    }

    auto host = ip::address::from_string(av[1]);
    auto port = std::stoi(av[2]);

    std::string b60(60, '0');
    std::string b32k(1024*32, '1');

    constexpr int n_req = 128;
    echo_test(host, port, n_req, 12321);
    echo_test(host, port, n_req, 32123);
    echo_test(host, port, n_req, std::string("==="));
    echo_test(host, port, n_req, b60);
    echo_test(host, port, n_req, b60 + b60 + b60 + b60);
    echo_test(host, port, n_req, b32k);
    echo_test(host, port, n_req, b32k + b32k + b32k + b32k);
    //echo_test(host, port, n_req, std::map<int,std::string>{{11,"Hello world"}});

    return 0;
}

