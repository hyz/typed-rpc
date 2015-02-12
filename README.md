# cpp-async-native-messaging

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

example:
=============
message table

```c++
typedef Message::tag<struct M1> Msg_Time;

typedef Message::Table<
    Message::Pair< Request<Msg_Time>, Response<std::string> >
> Message_table;
```

server side

```c++
struct Message_handle : service_def<Message_handle,Message_table> //::: server-side
{
    template <typename Reply>
    void operator()(Reply reply, Msg_Time) const
    {
        time_t ct = time(0);
        reply( std::string( ctime(&ct) ) );
    }

    Message_handle(ip::tcp::socket* s) : service_def<Message_handle,Message_table>(s) {}
};
```
client side

```c++
std::string get_time(ip::address host, unsigned short port)
{
    boost::asio::io_service io_s;
    Example::Client exc(io_s, host, port);

    std::string ret;
    auto save_time = [&ret](std::string& ts) {
        ret = std::move(ts);
    };
    exc.async_do(save_time, Example::Msg_Time());
    io_s.run();
    return std::move(ret);
}

```

see example.cpp

run test
=============

    test with gcc-4.8.
    $
    $ export BOOST_ROOT=boost_1_57_0
    $ b2
    $ bin/example -l 33333
    $ bin/example 127.0.0.1 33333 -t

