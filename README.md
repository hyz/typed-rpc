# cpp-remote-messaging

tcp socket request-response通信高级抽象，可类比RPC。
no protocol required, transfer native cpp datastructs, such as list, map etc.
network transparent.
simple, single header only file.

features:
=========
    * 表格描述通信协议格式
    * 数据支持c++任何类型(被Boost.Serialization支持)
    * 易用

dependencies：
=============
    * c++11 meta-programming
    * Boost.Asio
    * Boost.Serialization
    * Boost.Pool
    * Boost.Intrusive
    * Boost.Interprocess
    * Boost.MSM

example:
=============

```c++
typedef Message::tag<struct N0> Msg_Echo;
typedef Message::tag<struct N1> Msg_Time;
typedef Message::tag<struct N2> Msg_File;
typedef Message::Pair< Message::Request<>, Message::Response<> > ___reserved___;

using Message::Request;
using Message::Response;

typedef Message::Table<
    ___reserved___
  , ___reserved___
  , ___reserved___

  , Message::Pair< Request<Msg_Echo, UInt, UInt>, Response<UInt, UInt> >
  , Message::Pair< Request<Msg_Echo, UInt, UInt>, Response<UInt, UInt> >
  , Message::Pair< Request<Msg_Echo, UInt, std::string>, Response<UInt, std::string> >
  , ___reserved___
  , ___reserved___
  , ___reserved___

  , Message::Pair< Request<Msg_Time>, Response<std::string> >
  , ___reserved___
  , Message::Pair< Request<Msg_File,std::string>, Response<int,std::string> >
  , ___reserved___
> Message_table;

```

