# cpp-remote-messaging

async tcp request/response client & server, RPC-like.
mainly designed for exchange message between c++ server nodes.

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
    test with gcc-4.8.
    $ export BOOST_BUILD=/opt/boost
    $ b2
    $ bin/example

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

