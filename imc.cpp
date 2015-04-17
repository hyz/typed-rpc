#include "json.h"

struct jsmessage : json::object
{
    jsmessage() {}
    jsmessage(int cmd, int ec=0)
    {
        json::insert(*this) ("cmd", json::int_t(cmd)) ("error", json::int_t(ec))
            ;
    }

    json::object& body()
    {
        auto p = this->emplace("body", json::object());
        return json::ref<json::object>(p.first->second);
    }

    template <typename V>
    V& body(V&& v)
    {
        auto p = this->emplace("body", std::forward<V>(v));
        if (!p.second)
            p.first->second = std::forward<V>(v);
        return json::ref<V>(p.first->second);
    }

    void tag(UInt x) { json::insert(*this) ("tag", x); }

    std::string pack() const
    {
        // AUTO_CPU_TIMER(":pack:json:encode");
        union {
            uint32_t len;
            char s[sizeof(uint32_t)];
        } u;

        std::string tmp = json::encode(*this);
        u.len = htonl(tmp.size());
        //LOG << pfx << tmp.size() << tmp;

        return std::string(u.s, u.s+sizeof(u)) + tmp;
    }

    template <typename R>
    void unpack(R const& rng)
    {}
};

namespace yx {

template <typename Service_>
struct service_1 : msm::back::state_machine<Service_>
{
    typedef service_1 This;
    typedef boost::intrusive_ptr<This> pointer;

    boost::intrusive::list_member_hook<> list_hook_;
    boost::intrusive::unordered_set_member_hook<> set_hook_;
    time_t tplive_ = 0;
    unsigned int reference_count_ = 0;

    template <typename... T>
    service_1(T&&... t)
        : msm::back::state_machine<Service_>(std::forward<T>(t)...)
    {}

public:
    template <typename B>
    static This & downcast(B& x)
    {
        BOOST_STATIC_ASSERT(std::is_base_of<B,This>::value);
        return static_cast<This&>(x);
    };
    template <typename B>
    static This const& downcast(B const& x) { return downcast(const_cast<B&>(x)); };
    template <typename B>
    static pointer get_pointer(B const& x) { return pointer( & downcast(const_cast<B&>(x)) ); };
};

template <typename Service_>
struct service_list1 : singleton<service_list1<Service_>>
{
    typedef service_list1 This;
    typedef yx::service_1<Service_> Service;
    typedef boost::intrusive_ptr<Service> pointer;

    typedef boost::intrusive::unordered_multiset<Service, boost::intrusive::member_hook<Service,
                boost::intrusive::unordered_set_member_hook<>, &Service::set_hook_>> index_type;
    typedef boost::intrusive::list<Service, boost::intrusive::member_hook<Service,
                boost::intrusive::list_member_hook<>, &Service::list_hook_>> list_type;

    list_type list;
    index_type index;
    boost::object_pool<Service> pool_;

    template <size_t Size>
    static typename index_type::bucket_traits bucket_traits()
    {
        static typename index_type::bucket_type buckets_[Size];
        return {buckets_, Size};
    } //typename index_type::bucket_type bucket_traits

    service_list1(boost::asio::io_service& io_s)
        : index(bucket_traits<Service::Bucket_size>())//(typename index_type::bucket_traits(buckets_, Service::Bucket_size))
        , timer_(io_s)
    { }

    template <typename... T>
    static pointer create(T&& ... t)
    {
        auto& my = This::instance();
        pointer ptr( my.pool_.construct(std::forward<T>(t)...) );
        my.list.push_back(*ptr);
        return ptr; //pointer( lis.emplace(lis.end(), *ptr) );
    }

    static void destroy(Service const& m)
    {
        auto& my = This::instance();
        //! my.index.erase(my.index.iterator_to(m));
        my.list.erase(my.list.iterator_to(m));
    }

    template <typename K>
    static pointer find(K const& k)
    {
        auto& my = This::instance();
        auto it = my.index.find(k, typename Service::hash_key_func{}, typename Service::hash_key_equal{});
        return pointer( it==my.index.end() ? nullptr : it.operator->() );
    }
    //template <typename K>
    //static std::pair<iterator,iterator> equal_range(K const& k)
    //{
    //    return index.find(k, typename Service::hash_key_func{}, typename Service::hash_key_equal{});
    //}

    template <typename T>
    static void index_erase(T const& t)
    {
        auto& idx = This::instance().index;
        idx.erase( idx.iterator_to(Service::downcast(t)) );
    }

    template <typename T>
    static void index_insert(T& t)
    {
        auto& idx = This::instance().index;
        idx.insert( Service::downcast(t) );//.second;
    }

    static list_type const& get_list() 
    {
        auto& my = This::instance();
        return my.list;
    }

private:
    boost::asio::deadline_timer timer_;

    //static void check()
    //{
    //    auto& lis = This::instance().lis_;
    //    if (lis.empty()) {
    //        return;
    //    }
    //    enum { Ts=60*6 };
    //    time_t tpc = time(0);

    //    boost::intrusive::list<IMC_service>::iterator const b = lis.begin();
    //    boost::intrusive::list<IMC_service>::iterator it;
    //    while ( (it = lis.begin()) != b) {
    //        if (it->tplive_ + Ts < tpc) {
    //            lis.splice(lis.end(), lis, it);
    //            LOG << "expire" << it->connection_info();
    //            it->close();
    //        }
    //    }
    //}
    //struct getid {
    //    typedef unsigned int result_type;
    //    result_type operator()(IMC_service_ptr const& p) const { return p->uid_; }
    //};

    //typedef typename list_type::iterator list_iterator;
    //struct iterator //: list_iterator //, private std::function<void(iterator const&)>
    //{
    //    ~iterator() { This::decr_(*this); }

    //    template <typename B>
    //    iterator(B const& x) : ptr_(static_cast<Service*>(const_cast<B*>(&x))) //: list_iterator(iterator_to_(x))
    //        { This::incr_(*this); }
    //    //iterator(list_iterator it) : list_iterator(it) { This::incr_(*this); }
    //    iterator(iterator const& rhs) : ptr_(static_cast<Service*>(const_cast<B*>(&x))) //list_iterator(rhs)
    //        { This::incr_(*this); }
    //    iterator& operator=(iterator const& rhs) {
    //        if (this!=&rhs) {
    //            This::decr_(*this);
    //            list_iterator::operator=(rhs);
    //            This::incr_(*this);
    //        }
    //        return *this;
    //    }
    ////private:
    //    //template <typename B>
    //    //static list_iterator iterator_to_(B const& b) {
    //    //    BOOST_STATIC_ASSERT(std::is_base_of<B,Service>::value);
    //    //    return list_type::s_iterator_to(b);
    //    //}
    //    Service* ptr_ = 0;
    //};
    //friend struct iterator;

    //static void incr_(iterator& it) {
    //    ++(it->operator->()->reference_count_);
    //}
    //static void decr_(iterator& it) {
    //    if (--(it->operator->()->reference_count_) == 0) {
    //        auto& my = This::instance();
    //        my.index.erase(it->operator*());
    //        my.list.erase(it);
    //    }
    //}
};
}

template <typename S>
inline void intrusive_ptr_add_ref(yx::service_1<S> * m)
{
    ++m->reference_count_;
}
template <typename S>
inline void intrusive_ptr_release(yx::service_1<S> * m)
{
    if (--m->reference_count_ == 0) {
        yx::service_list1<S>::destroy(*m);
    }
}

struct IMC_service_ : msm::front::state_machine_def<IMC_service_>
{
    typedef IMC_service_ This;
    typedef yx::service_1<IMC_service_> Service;
    typedef boost::intrusive_ptr<Service> pointer;

    enum struct Cmd : int {
          None
        , list_online=71, count_onlines=72
        , heartbeat=91, ack=95
        , auth=99
        , sendmsg=205
        , set_devtok=211
        , enter_chatroom=215, exit_chatroom=216
    };

    struct Ev_close    {};
    struct Ev_read     {};
    struct Ev_writeall {};
    struct Ev_response {};

    tcp::socket socket;
    UInt uid_ = 0;

    struct message_rx
    {
        uint32_t len = 0;
        std::vector<char> buf;
        int cmd = 0;
        json::object body;
    } rx_;

    IMC_service_(tcp::socket* sk) : socket(std::move(*sk))
    {}

    struct rsphandle_ignore {
        template <typename...T> void operator()(T&&...) const {}
    };
    struct rsphandle_auth {
        pointer ptr_;
        void operator()(int result);
    };
    struct rsphandle_sendmsg {
        pointer ptr_;
        template <typename V> void operator()(int result, UInt msgid, std::string const& sid)
        {
            jsmessage msg(Cmd::sendmsg);
            json::insert(msg.body())("sid",sid)("result",result);
            auto& m = *ptr_;
            m.send_message(msg);
            m.process_event(Ev_response());
        }
    };
    struct rsphandle_devtok {
        pointer ptr_;
        template <typename V> void operator()(int result, UInt msgid, std::string const& sid)
        {
            jsmessage msg(Cmd::set_devtok);
            json::insert(msg.body()) ("sid",sid) ("result",result) ;
            auto& m = *ptr_;
            m.send_message(msg);
            m.process_event(Ev_response());
        }
    };
    struct rsphandle_enter_chatroom {
        pointer ptr_;
        template <typename V> void operator()(int result, UInt msgid, std::string const& sid)
        {
            jsmessage msg(Cmd::enter_chatroom);
            json::insert(msg.body())("sid",sid)("result",result);
            auto& m = *ptr_;
            m.send_message(msg);
            m.process_event(Ev_response());
        }
    };
    struct rsphandle_exit_chatroom {
        pointer ptr_;
        template <typename V> void operator()(int result, UInt msgid, std::string const& sid)
        {
            jsmessage msg(Cmd::exit_chatroom);
            json::insert(msg.body())("sid",sid)("result",result);
            auto& m = *ptr_;
            m.send_message(msg);
            m.process_event(Ev_response());
        }
    };

    struct handle_read {
        pointer ptr_;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            auto& m = *ptr_; // ptr->handle_read(*ptr, ec, bytes_transferred);
            if (ec) {
                LOG << ec << ec.message();
                m.process_event(Ev_close());
                return;
            }
            m.process_event(Ev_read());
        }
    };
    struct handle_write {
        pointer ptr_;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            auto& m = *ptr_; // ptr->handle_read(*ptr, ec, bytes_transferred);
            if (ec) {
                LOG << ec << ec.message();
                m.process_event(Ev_close());
                return;
            }
            if (!m.queue_send_is_empty()) {
                // TODO
                //auto bufs[] = { as_buffer(out.len), sbuf.data() };
                //boost::asio::async_write(m.socket, bufs, *this);
                return;
            }
            m.process_event(Ev_writeall());
        }
    };

    struct Wait_header : msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {
            boost::asio::async_read(m.socket
                    , as_buffer(m.rx_.len)
                    , handle_read{ Service::get_pointer(m) });
            //transfer_at_least
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_body : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {
            m.rx_.buf.resize(ntohl(m.rx_.len));
            boost::asio::async_read(m.socket
                    , boost::asio::buffer(m.rx_.buf)
                    , handle_read{ Service::get_pointer(m) });
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_response : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_write : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct _Switch_next : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Running : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&) {}
        template <class Ev, class M> void on_exit(Ev const&, M&);
    };
    struct Closing : msm::front::interrupt_state<Ev_writeall> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {} // if (m.writq.empty()) { m.process_event(Ev_writeall()); }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Closed : msm::front::terminate_state<> {
        template <class Ev, class M> void on_entry(Ev const&, M& m)
        {
            boost::system::error_code ec;
            m.socket.close(ec); //indexer::destroy();
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };

    struct Has_write_data {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts)
        {
            return (!m.queue_send_is_empty());
        }
    };
    struct Is_request {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts)
        {
            return m.rx_.cmd >= 99;
        }
    };
    
    struct Process_message {
        template <class Ev, class M, class S, class T>
        void operator()(Ev const& ev, M& m, S&, T&) {
            doit(m);
        }
        template <class M> void doit(M& m);
    };

    //typedef Wait_header initial_state;
    typedef mpl::vector<Wait_header,Running> initial_state;

    struct transition_table : mpl::vector<
        Row< Wait_header   , Ev_read     , Wait_body     , none            , none           >
      , Row< Wait_body     , Ev_read     , _Switch_next  , Process_message , none           >
      , Row< Wait_response , Ev_response , Wait_write    , none            , none           >
      , Row< Wait_write    , Ev_writeall , Wait_header   , none            , none           >
      , Row< _Switch_next  , none        , Wait_header   , none            , none           >
      , Row< _Switch_next  , none        , Wait_write    , none            , Has_write_data >
      , Row< _Switch_next  , none        , Wait_response , none            , Is_request     >
        /// ///
      , Row< Running       , Ev_close    , Closed        , none            , none           >
      , Row< Running       , Ev_close    , Closing       , none            , Has_write_data >
      , Row< Closing       , Ev_writeall , Closed        , none            , none           >
    > {};

    template <class Ev, class M>
    void no_transition(Ev const& ev, M& fsm, int state)
    {
        ////(std::is_same<EvWrite,typename std::decay<ev>::type>::value) {}
    }

    template <class Ev, class M> void on_entry(Ev const& ev, M& m)
    {
        ;
    }
    template <class Ev, class M> void on_exit(Ev const&, M&) {}

    struct acknowledge
    {
        std::list<std::pair<UInt,std::function<void(int, UInt)>>> fv_;
        template <typename Fn>
        void operator()(UInt mid, Fn&& fn)
        {
            fv_.push_back(std::make_pair(mid,std::forward<Fn>(fn)));
        }
        void pop_(UInt msgid, std::string const& sid)
        {
            if (fv_.empty() || msgid != fv_.front().first) {
                return;
            }
            auto p = std::move(fv_.front());
            fv_.pop_front();
            p.second(0, msgid);
        }
    };
    acknowledge ack;

    This& send_message(std::string const& msg)
    {
        return *this;
    }
    This& send_message(jsmessage const& msg)
    {

        return *this;
    }

    bool queue_send_is_empty() const {
        return 1; // TODO
    }

    std::string connection_info() const {
        return std::to_string(uid_);
    }

    //void close() { derived().process_event(Ev_close()); }

public:
    BOOST_STATIC_CONSTANT(int, Bucket_size=10000*50);
    typedef boost::hash<UInt> hash_key_func;
    struct hash_key_equal {
        bool operator()(UInt u, This const& rhs) const { return u == rhs.uid_; }
    };
    friend bool operator==(const This &l, const This &r) {  return l.uid_ == r.uid_;   }
    friend std::size_t hash_value(const This &v) {  return boost::hash_value(v.uid_); }
};

template <class Ev, class M>
void IMC_service_::Running::on_exit(Ev const&, M& m)
{
    if (m.uid_ > 0) {
        yx::service_list1<This>::index_erase(m);
        Message_bus::offline(m.uid_, rsphandle_ignore{});
    }
}

void IMC_service_::rsphandle_auth::operator()(int result)
{
    auto& m = *ptr_;

    jsmessage msg(static_cast<int>(Cmd::auth));
    json::insert(msg.body())("result",result);
    m.send_message(msg);

    if (result == 0) {
        yx::service_list1<This>::index_insert(m);
        Message_bus::online(m.uid_, rsphandle_ignore{});
        m.process_event(Ev_response());
    } else {
        m.process_event(Ev_close());
    }
}

template <class M>
void IMC_service_::Process_message::doit(M& m)
{
    m.rx_.body.clear();

    boost::optional<json::object> jv0 = json::decode<json::object>( m.rx_.buf );
    if (!jv0) {
        LOG << "decode-fail";
        return;
    }
    json::object& jv = jv0.value();
    int cmd = json::ref<int>(jv, "cmd");

    json::object& body = json::ref<json::object>(jv, "body");
    boost::optional<UInt> tag; {
        auto it = body.iterator_to("tag"); //as<std::string>(body, "tag");
        if (it != body.end()) {
            tag = json::ref<int>(it->second);
            body.erase(it);
        }
    }

    switch (static_cast<Cmd>(cmd)) {
    //case Cmd::list_online:
    //    {
    //        jsmessage msg(cmd);
    //        json::array& jv = msg.body(json::array());
    //        BOOST_FOREACH(auto & x, yx::service_list1<This>::get_list()) {
    //            jv.push_back( x.connection_info() );
    //        }
    //        m.send_message(msg);
    //    }
    //    break;
    //case Cmd::count_onlines:
    //    {
    //        jsmessage msg(cmd);
    //        auto& lis = yx::service_list1<This>::get_list();
    //        msg.emplace("count", lis.size());
    //        m.send_message(msg);
    //    }
    //    break;
    case Cmd::heartbeat:
        break;
    case Cmd::ack:
        m.ack.pop_(json::ref<int>(body,"msgid"), json::ref<std::string>(body,"sid"));
        break;
    case Cmd::auth:
        {
            UInt uid = json::ref<int>(body, "uid");
            auto& tok = json::ref<std::string>(body, "token");
            Authentication::au(uid, tok, rsphandle_auth{ Service::get_pointer(m) });
        }
        break;
    case Cmd::sendmsg:
        break;
    case Cmd::set_devtok:
        break;
    case Cmd::enter_chatroom:
        break;
    case Cmd::exit_chatroom:
        break;
    default: 
        LOG << cmd << "unknown";
        break;
    }
}

// typedef service_list2<service_<Push::Message_table, Push::S::Message_handle>> Push_server;

static void handle_imc_accept(Acceptor* a, boost::system::error_code ec)
{
    if (ec) {
        LOG << ec << ec.message();
        return;
    }
    auto it = yx::service_list1<IMC_service_>::create(&a->socket);
    it->start(); //::: start

    a->async_accept(boost::bind(&handle_imc_accept, a, placeholders::error));
}

int server_main(tcp::endpoint ep1, tcp::endpoint ep2)//(int argc, char* const argv[])
{
    boost::asio::io_service io_s;

    //yx::service_list1<IMC_service_> imc_services(io_s);
    //Acceptor imca(io_s, ep2);
    //imca.async_accept(boost::bind(&handle_imc_accept, &imca, placeholders::error));

    std::cout << io_s.run() <<"\n";
    return 0;
}

