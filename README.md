# Typed rpc-linke messaging

Async tcp request/response client & server, RPC-like.
Mainly designed for exchange message between c++ server nodes.

features:
=========
    * no protocol required, transfer native cpp datastructs, such as list, map etc.
    * table declared exchange data
    * network transparent.
    * simple, single header only file.

dependencies：
=============
    * c++11 meta-programming
    * Boost.Asio
    * Boost.Serialization
    * Boost.Pool
    * Boost.Intrusive
    * Boost.Interprocess
    * Boost.MSM
    * ...

Echo example:
=============
Message table

```c++

typedef Message::Table<
    ___reserved___
  , Message::Pair< Request<std::string>, Response<std::string> >
  , Message::Pair< Request<int, std::string>, Response<int, std::string> >
  , Message::Pair< Request<std::map<int,std::string>>, Response<std::map<int,std::string>> >
> Message_table;
```

Server side

```c++
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
```

client side

```c++
template <typename Data>
void echo_test(ip::address host, unsigned short port, int n_req, Data&& data)
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

```

see examples/echo.cpp

run test
=============

    test with gcc-4.8.
    $
    $ cd examples
    $ export BOOST_ROOT=~/boost_1_57_0
    $ b2
    $ bin/echo -l 33333
    $ bin/echo 127.0.0.1 33333
    $

