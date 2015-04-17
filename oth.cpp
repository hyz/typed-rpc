using namespace Message;

typedef Message::Table<
  //Message::Def< Request<void             >, Response<int> >
    Message::Def< Request<UInt, std::string>, Response<int> >
> Authentication_message_table;

typedef Message::tag<struct ON > TOnline;
typedef Message::tag<struct OFF> TOffline;

typedef Message::Table<
  //Message::Def< Request<void             >, Response<int> >
    Message::Def< Request<UInt    >, Response<int> > // user online
  , Message::Def< Request<UInt,int>, Response<int> > // user offline
> Message_bus_message_table;

struct Authentication : agent_def<Authentication,Authentication_message_table> , singleton<Authentication>
{
    typedef agent_def<Authentication,Authentication_message_table> base;

    template <class Fn>
    static void au(UInt uid, std::string tok, Fn&& fn)
    {
        instance().async_do(fn, uid, tok);
    }

    Authentication(boost::asio::io_service& io_s, ip::address ipa, unsigned short port)
        : base(boost::ref(io_s), ipa, port)
    {}
};

struct Message_bus : agent_def<Message_bus,Message_bus_message_table> , singleton<Message_bus>
{
    typedef agent_def<Message_bus,Message_bus_message_table> base;

    template <class Fn>
    static void online(UInt uid, Fn&& fn)
    {
        instance().async_do(fn, uid);
    }

    template <class Fn>
    static void offline(UInt uid, Fn&& fn)
    {
        instance().async_do(fn, uid, 1);
    }

    Message_bus(boost::asio::io_service& io_s, ip::address ipa, unsigned short port)
        : base(boost::ref(io_s), ipa, port)
    {}
};

//struct Message_push : msm::back::state_machine<Multic<Push_messages>>
//                        , singleton<Message_push>
//{
//    typedef Multic<Push_messages> base_t;
//    typedef msm::back::state_machine<base_t> SMac;
//    BOOST_STATIC_ASSERT(std::is_base_of<base_t,SMac>::value);
//
//    template <class Fn>
//    static void finish(UInt uid, UInt msgid, std::string const& sid, Fn fn)
//    {
//        instance().async_do(query(fn, uid, msgid, sid));
//    }
//
//    Message_push(boost::asio::io_service& io_s, ip::address ipa, unsigned short port)
//        : SMac(boost::ref(io_s), ipa, port)
//    {}
//};


