#include <boost/predef.h> // BOOST_ENDIAN_BIG_BYTE
#include <boost/tuple/tuple.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/lexical_cast.hpp>
#include "singleton.h"

namespace multi_index = boost::multi_index;
namespace placeholders = boost::asio::placeholders;
namespace ip = boost::asio::ip;
using ip::tcp;

typedef unsigned int UInt;

struct int3byte_flag1byte
{
    union {
        uint32_t int3;
        uint8_t  bytes[4];
    } u_ = {0};
#if defined(BOOST_ENDIAN_LITTLE_BYTE)
    enum { Ix=3 };
#elif defined(BOOST_ENDIAN_BIG_BYTE)
    enum { Ix=0 };
#else
#  error "unknown endianness"
#endif

    template <int X> bool b8(bool y) {
        BOOST_STATIC_ASSERT(X<8);
        y ? (u_.bytes[Ix] |= (1<<X)): (u_.bytes[3] &= ~(1<<X));
    }
    template <int X> bool b8() const {
        BOOST_STATIC_ASSERT(X<8);
        return (u_.bytes[Ix] & (1<<X));
    }
    void b8reset() {
        u_.bytes[Ix] = 0;
    }

    uint32_t u3_decr() { BOOST_ASSERT(u3()>0); return --u_.int3; }
    uint32_t u3_incr() { BOOST_ASSERT(u3()<0x0ffffff); return ++u_.int3; }
    uint32_t u3() const { return (u_.int3 & 0x00ffffff); }
};

//basic_socket_acceptor(boost::asio::io_service& io_service, const endpoint_type& endpoint, bool reuse_addr = true)
struct Acceptor : tcp::acceptor
{
    tcp::socket socket_;

    Acceptor(boost::asio::io_service& io_s, tcp::endpoint endp)
        : tcp::acceptor(io_s, endp, true)
        , socket_(io_s)
    {
        open(endp.protocol());
        set_option(tcp::acceptor::reuse_address(true));
        bind(endp);
        listen();
    }

    template <typename H>
    void async_start(H&& h)
    {
        this->async_accept(socket_, std::forward<H>(h));
    }
};

struct resolver : tcp::resolver
{
    typedef resolver this_type;

    template <typename H>
    void async_query(std::string host, H&& h)
    {
        return async_query(host, 0, std::forwared<H>(h));
    }

    template <typename H>
    void async_query(std::string host, bool ignore_cache, H&& h)
    {
        if (ignore_cache) {
            auto p = cbs_.emplace(host, std::vector<callback_t>());
            p.first->second.push_back(h);
            resolv_(host);
            return;
        }

        auto it = resolved_.find(host);
        if (it != resolved_.end()) {
            it->tp_queryed_ = time(0);

            // relocate it;

            auto addr = it->addr;
            get_io_service().post([h,addr](){
                        h(boost::system::error_code(), addr);
                    });

            if (it->tp_resolved_ + 60*60 < time(0)) {
                return;
            }
        }

        auto p = cbs_.emplace(host, std::vector<callback_t>());
        p.first->second.push_back(h);
        resolv_(host);
    }

    void resolv_(std::string& host)
    {
        tcp::resolver::query q(host, "");
        async_resolve(q, [this,host](boost::system::error_code ec, tcp::resolver::iterator r_it){
                    handle_resolv(host, ec, r_it);
                });
    }

    void _check_time(time_t tpc)
    {
        auto& idx = boost::get<1>(resolved_);
        while (!idx.empty()) {
            auto& x = idx.front();
            if (x.tp_queryed_ + 60*60*4 < tpc) {
                break;
            }
            idx.pop_front();
        }
    }

    resolver(boost::asio::io_service& io_s)
        : tcp::resolver(io_s)
        , timer(io_s)
    {}

////// //////
    void handle_resolv(std::string host, boost::system::error_code ec, tcp::resolver::iterator it_r)
    {
        ip::address addr;
        if (!ec) {
            auto a = *it_r;
            addr = a.address();

            auto p = resolved_.emplace(host, addr);
            if (!p.second) {
                resolved_.modify(p.first, [&addr](record& rec){
                            rec.addr = addr;
                            rec.tp_resolved_ = time(0);
                        });
            }
        }
        auto it = cbs_.find(host);
        if (it != chs_.end()) {
            for (auto& fn : it->second) {
                fn(ec, addr);
            }
            cbs_.erase(it);
        }
    }

    struct record
    {
        std::string host; // tcp::resolver::query query; //(args.host_, args.port_));
        ip::address addr; // tcp::endpoint endp;
        time_t tp_resolved_;
        time_t tp_queryed_;
    };

    boost::multi_index_container<
        record,
        multi_index::indexed_by<
            multi_index::hashed_unique<multi_index::member<record, std::string const&, &record::host>>
          , multi_index::sequenced<>
        >
    > resolved_;

    typedef std::function<void(ip::address const&)> callback_t;
    std::unordered_map<std::string,std::vector<callback_t>> cbs_;

    boost::asio::deadline_timer timer;
};

// struct Connector : resolver
// {
//     typedef Connector this_type;
// 
//     template <typename H>
//     void async_connect(tcp::socket& sk, std::string const& host, int port, H&& h)
//     {
//         tcp::socket sk ...; 
// 
//         async_resolve(host, [this,&sk,port,h](boost::system::error_code ec, ip::address ipa){
//                     if (ec) {
//                         h(ec, sk);
//                     } else {
//                         this->async_connect(sk, tcp::endpoint(ipa,port), h);
//                     }
//                 });
//     }
// 
//     template <typename H>
//     void async_connect(tcp::socket& sk, ip::tcp::endpoint endp, H&& h)
//     {
//         sk.async_connect(endp, [this,h](boost::system::error_code ec){
//                     ;
//                 });
//     }
// 
//     Connector(boost::asio::io_service& io_s)
//         : resolver(io_s)
//     {}
// 
//     struct connection : tcp::socket
//     {
//         tcp::endpoint endp;
//     };
// 
//     boost::multi_index_container<
//         connection,
//         multi_index::indexed_by<
//             multi_index::hashed_unique<multi_index::member<connection, tcp::endpoint const&, &connection::endp>>
//           , multi_index::sequenced<>
//         >
//     > connections_;
// };

template <typename Socket_object>
struct Socket_list : singleton<Socket_list<Socket_object>>
        , boost::multi_index_container<
            Socket_object,
            multi_index::indexed_by<
                multi_index::hashed_non_unique<typename Socket_object::key_extractor>
              , multi_index::sequenced<>
            >
        >
{
    typedef Socket_list<Socket_object> this_type;
    typedef boost::multi_index_container<
            Socket_object,
            multi_index::indexed_by<
                multi_index::hashed_non_unique<typename Socket_object::key_extractor>
              , multi_index::sequenced<>
            >
        > container;
    typedef typename container::iterator raw_iterator;

    struct iterator_t : raw_iterator //, private std::function<void(iterator_t const&)>
    {
        // pointer get_pointer() const { return const_cast<pointer>(raw_iterator::operator->()); }
        this_type* container_; // typedef std::function<void(iterator_t const&)> Deleter;

        // extern void iterator_ref_incr(this_type&, raw_iterator);
        // extern void iterator_ref_decr(this_type&, raw_iterator);
        void incr() const { iterator_ref_incr(*container_, *static_cast<raw_iterator*>(this)); }
        void decr() const { iterator_ref_decr(*container_, *static_cast<raw_iterator*>(this)); }

        // template <int Io> auto buffer() const -> boost::element<0,Socket_object::bufs_type>::type& { return boost::get<Io>(sock().bufs); }

        ~iterator_t() {
            decr();
        }
        //!iterator_t() {}
        iterator_t(raw_iterator it, this_type* t) : raw_iterator(it) {
            container_ = t;
            incr();
        }
        iterator_t(iterator_t const& rhs) : raw_iterator(rhs) {
            container_ = rhs.container_;
            incr();
        }
        iterator_t& operator=(iterator_t const& rhs) {
            if (this!=&rhs) {
                decr();
                raw_iterator& b = *this; // Deleter& d = *this;
                b = rhs; // d = rhs;
                container_ = rhs.container_;
                incr();
            }
            return *this;
        }
    };
    typedef iterator_t iterator;

    auto emplace(tcp::socket&& s) -> std::pair<iterator_t, bool>
    {
        auto p = container::emplace(std::move(s));
        if (p.second)
            ;
        return std::make_pair(iterator_t(p.first,this), p.second);
    }
};

// template <typename Derived>
//inline boost::asio::mutable_buffer mutable_buffer(uint32_t& n)
//{
//    return boost::asio::mutable_buffer(static_cast<void*>(&n), sizeof(uint32_t));
//}
//inline boost::asio::const_buffer const_buffer(uint32_t const& n)
//{
//    return boost::asio::const_buffer(static_cast<void const*>(&n), sizeof(uint32_t));
//}
inline boost::asio::mutable_buffer as_buffer(uint32_t& n)
{
    return boost::asio::mutable_buffer(static_cast<void*>(&n), sizeof(uint32_t));
}
inline boost::asio::const_buffer as_buffer(uint32_t const& n)
{
    return boost::asio::const_buffer(static_cast<void const*>(&n), sizeof(uint32_t));
}

struct Fx_read  { BOOST_STATIC_CONSTANT(int,N=0); };
struct Fx_write { BOOST_STATIC_CONSTANT(int,N=1); };
struct Fx_eof   { BOOST_STATIC_CONSTANT(int,N=2); };
struct Fx_close { BOOST_STATIC_CONSTANT(int,N=3); };
struct Fx_error { BOOST_STATIC_CONSTANT(int,N=4); };
// enum class bidx { reading=0, writing=1, eof, close, error };

//#include <boost/archive/text_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>
//typedef boost::archive::text_oarchive oArchive;
//typedef boost::archive::text_iarchive iArchive;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
typedef boost::archive::binary_oarchive oArchive;
typedef boost::archive::binary_iarchive iArchive;

template <typename PTable>
struct Message_table //: Socket_stat //<Derived>
{
    typedef Message_table/*<Derived>*/ this_type;
    typedef std::function<void(boost::system::error_code)> callback_type;

    Derived& derived() { return static_cast<Derived*>(this); }
    tcp::socket& socket() { return derived()->socket(); }

    template <typename Const_buf, typename Callback>
    void async_send(Const_buf const& buf, Callback cb)
    {
        size_t siz = buf.size();
        if (siz == 0) {
            return;
        }
        auto& obufs = bufs.get<1>();
        bool empty = obufs.empty();
        obufs.emplace_back(buf, std::move(cb));
        if (empty) {
            async_send_next_(boost::system::error_code());
        }
        //derived()->report<Fx_write>(siz);
    }

    template <typename Callback>
    void async_recv(Callback cb)
    {
        auto& buf = bufs.get<0>();
        buf.callback = std::move(cb);
        buf.len = 0;
        boost::asio::async_read(socket(), as_buffer(buf.len),
                boost::bind(&this_type::handle_read_size, this, placeholders::error, placeholders::bytes_transferred));
    }

    // void reset() { bufs = boost::tuple<Input,Output>(); }

private:
    struct Base_buf : std::tuple<boost::asio::streambuf,callback_type> {
        void clear() {
            auto& sb = std::get<0>(*this);
            sb.comsume(sb.size());
            std::get<1>(*this) = callback_type();
        }
        boost::asio::streambuf & streambuf() { return std::get<0>(*this); }
        template <typename...T> callback(T&&... t) { return std::get<1>(std::forward<T>(t)...); }
    };
    struct Input : boost::asio::streambuf {
        uint32_t len = 0;
        //this_type* parent() const { return boost::intrusive::get_parent_from_member(this, &this_type::input); }
    };
    struct Output : std::list<Base_buf> {
        uint32_t len = 0;
        //this_type* parent() const { return boost::intrusive::get_parent_from_member(this, &this_type::writq); }
    };

    boost::tuple<Input,Output> bufs; // boost::element<0, bufs_type>::type;

    void handle_read(boost::system::error_code const& ec, size_t bytes_transferred)
    {
        auto& input = std::get<0>(this->bufs);
        //derived()->report<Fx_read>(ec, bytes_transferred);

        iArchive ar(input, boost::archive::no_header);
        if (ec) {
            // input(ec, ar);
        } else {
            ;
        }
        input.clear();

        //boost::asio::async_read( *this, mutable_buffer(buf.len),
        //        boost::bind(&this_type::handle_read_size, this, placeholders::error, placeholders::bytes_transferred));
    }

    void handle_read_size(boost::system::error_code const& ec/*, size_t bytes_transferred*/)
    {
        BOOST_ASSERT(!intput.callback.empty());
        if (ec) {
            //derived()->report<Fx_read>(ec, bytes_transferred);
            input.callback(ec, boost::asio::streambuf());
        } else {
            boost::asio::async_read(socket()
                    , input.prepare(ntohl(input.len))
                    , boost::bind(&this_type::handle_read, placeholders::error));
        }
    }

    void handle_write(boost::system::error_code ec/*, size_t bytes_transferred*/)
    {
        auto& outs = std::get<1>(this->bufs);
        BOOST_ASSERT(!outs.empty());
        BOOST_ASSERT(!outs.callback.empty());
        //derived()->report<Fx_write>(ec, bytes_transferred);
        if (!ec) {
            auto tmp = std::move( outs.front() );
            outs.pop_front();
            // boost::asio::streambuf& sb = tmp.get<0>();
            std::get<1>(tmp)()(ec);
        }
        if (!outs.empty()) {
            async_send_next_(std::get<0>(outs.front()));
        }
    }

    template <typename Buf>
    void async_send_next_(Buf const& buf)
    {
        obufs.len = htonl(buf.size());
        boost::array<boost::asio::const_buffer,2> outs{ as_buffer(obufs.len), buf.data() };
        boost::asio::async_write(socket(), outs,
                boost::bind(&this_type::handle_write, this, placeholders::error, placeholders::bytes_transferred));
    }

    void async_send_next(boost::system::error_code ec)
    {
        auto& outs = std::get<1>(this->bufs);
        if (!outs.empty()) {
            auto& p = outs.front();
            async_send_next_(std::get<0>(p));
        }
    }
};

struct Socket_stat : int3byte_flag1byte //stat_;
{
    void reset() { b8reset(); }

    template <int X> void flag() const { return b8<X>(); }
    template <int X> void flag(bool y) { b8<X>(y); }

    template <typename Fx>
    void report(boost::system::error_code ec, size_t bytes_transferred) const
    {
        BOOST_ASSERT(thiz);
        if (ec) {
            // thiz->stat_.b8<bidx::error>(1);
            // if (ec == boost::asio::error::eof) {
            //     thiz->stat_.b8<bidx::eof>(1);
            // }
        }
    }

    template <typename Fx>
    void report(size_t siz) const
    {
    }
};

struct Socket_client_list
{
    template <typename...T, typename H>
    void async_query(tcp::endpoint endp, std::tuple<T...> const& q, H&& h)
    {
        ;
    }
};

template <typename T>
void handle_write(T* t, boost::system::error_code ec, size_t bytes_transferred)
{
    t->process_event(Ev_write<T>{t, ec});
}

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <type_traits>

namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;

namespace { // anonymous

template <typename...T> using tuple = std::tuple<T...>;
template <typename...T> using Message_table_def = std::tuple<T...>;
template <typename...T> using Message_def = std::tuple<T...>;
typedef Message_table_def <
          Message_def< tuple<int, string        >, tuple<int> >
        , Message_def< tuple<int, string, string>, tuple<int> >
    > Mtable_def;

struct IMC_service : boost::noncopyable // : msm::front::state_machine_def<IMC_service>
{
    typedef IMC_service this_type;

    tcp::socket socket;

    UInt uid_;
    time_t tpa_;

    IMC_service(tcp::socket && sk);

    struct message_rx
    {
        uint32_t len = 0;
        std::vector<char> buf;
        int cmd = 0;
        json::object body;
    } rx_;

    // tx_;

    //Message_table<Mtable_def> message_table;
    //friend struct Message_handler;
};

//struct IMC_service::Message_handler
//{
//    Socket_server* thiz;
//
//    std::vector<char> operator()(int, string) const
//    {
//        std::vector<char> out;
//
//        return std::move(out);
//    }
//    std::vector<char> void operator()(int, string, string) const
//    {
//        std::vector<char> out;
//
//        return std::move(out);
//    }
//};

#include <boost/intrusive_ptr.hpp>

struct IMC_fsm_;
typedef msm::back::state_machine<IMC_fsm_> IMC_fsm;
typedef boost::intrusive_ptr<IMC_fsm_t> IMC_fsm_ptr;

//struct IMC_fsm_t;
//struct IMC_fsm_t: msm::back::state_machine<IMC_fsm_> IMC_fsm
//                  , boost::intrusive::list_base_hook<>
//                  , boost::noncopyable
//{};

#include <boost/intrusive/list.hpp>

struct IMC_fsm_ : msm::front::state_machine_def<IMC_fsm_>
                  , boost::intrusive::list_base_hook<>
                  , boost::noncopyable
{
    // boost::intrusive::list_member_hook<> list_hook_;
    typedef IMC_fsm_ this_type;

    struct Ev_read     {};
    struct Ev_writeall {};
    struct Ev_response {};

    // typedef typename Socket_list<IMC_service>::iterator_t iterator;
    // {{{
    tcp::socket socket;

    UInt uid_ = 0;
    time_t tpa_ = 0;

    struct message_rx
    {
        uint32_t len = 0;
        std::vector<char> buf;
        int cmd = 0;
        json::object body;
    } rx_;

    // }}}

    IMC_fsm_(tcp::socket && sk) : socket(std::move(sk)) {
        tpa_ = time(0);
        s_lis_.push_front(*this);
    }
    ~IMC_fsm_() {
        s_lis_.erase(s_lis_.iterator_to(*this));
    }

    struct read_handler {
        IMC_fsm_ptr ptr;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            auto& m = *ptr; // ptr->handle_read(*ptr, ec, bytes_transferred);
            if (ec) {
                ;; //TODO
                return;
            }
            m.process_event(Ev_read());
        }
    };
    struct write_handler {
        IMC_fsm_ptr ptr;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            auto& m = *ptr; // ptr->handle_read(*ptr, ec, bytes_transferred);
            if (ec) {
                ;; //TODO
                return;
            }
            if (!m.writq.empty()) {
                //auto bufs[] = { const_buffer(out.len), sbuf.data() };
                boost::asio::async_write(m.socket, outs, *this);
                return;
            }
            m.process_event(Ev_writeall());
        }
    };

    struct response_auth {
        IMC_fsm_ptr ptr;
        void operator()(int result);
    };
    struct response_sendmsg {
        IMC_fsm_ptr ptr;
        template <typename V> void operator()(int result, Uint msgid, std::string const& sid)
        {
            auto& m = *ptr; // if (msgid == 0) { return; }
            json::object body;
            json::insert(body)("sid",sid)("result",result);
            m.append_writq(imcprotocol::pack(Msgcmd_sendmsg, body));
            m.process_event(Ev_response());
        }
    };

    struct Read_head : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {
            boost::asio::async_read(m.socket
                    , mutable_buffer(m.rx_.len)
                    , read_handler{ IMC_fsm_ptr(&m) });
            //transfer_at_least
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Read_body : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {
            m.rx_.buf.resize(ntohl(m.rx_.len));
            boost::asio::async_read(m.socket
                    , boost::asio::buffer(m.rx_.buf)
                    , read_handler{ IMC_fsm_ptr(&m) });
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Response_Wait : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_write : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {}
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
            m.socket.close(ec);
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };

    struct Has_write_data {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts)
        {
            return (!m.writq.empty());
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
    struct Delete_index {
        template <class Ev, class M, class S, class T>
        void operator()(Ev const& ev, M& m, S&, T&);
    };

    //typedef Read_head initial_state;
    typedef mpl::vector<Read_head,Running> initial_state;

    struct transition_table : mpl::vector<
         Row< Read_head     , Ev_read     , Read_body     , none            , none           >
       , Row< Read_body     , Ev_read     , _Switch_next  , Process_message , none           >
       , Row< Response_Wait , Ev_response , Wait_write    , none            , none           >
       , Row< Wait_write    , Ev_writeall , Read_head     , none            , none           >
       , Row< _Switch_next  , none        , Read_head     , none            , none           >
       , Row< _Switch_next  , none        , Wait_write    , none            , Has_write_data >
       , Row< _Switch_next  , none        , Response_Wait , none            , Is_request     >
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

    static boost::intrusive::list<IMC_fsm> s_lis_;
};

boost::intrusive::list<IMC_fsm> IMC_fsm_::s_lis_;

struct IMC_uid {
    typedef unsigned int result_type;
    result_type operator()(IMC_fsm_ptr const& p) const { return p->uid_; }
};

boost::multi_index_container<
            IMC_fsm_ptr,
            multi_index::indexed_by<
                multi_index::hashed_non_unique<IMC_uid>
              // , multi_index::sequenced<>
            >
        > imc_index_;

boost::object_pool<IMC_fsm> imc_pool_;

template <class Ev, class M> void IMC_fsm_::Running::on_exit(Ev const&, M&)
{
    if (m.uid_ > 0) {
        imc_index_.erase(m.uid_);
    }
}

void IMC_fsm_::response_auth::operator()(int result)
{
    auto& m = *ptr;
    json::object body;
    json::insert(body)("result",result);
    m.append_writq(imcprotocol::pack(Msgcmd_auth, body));

    if (result == 0) {
        imc_index_;
        m.process_event(Ev_response());
    } else {
        m.process_event(Ev_close());
    }
}

template <class M>
void IMC_fsm_::Process_message::doit(M& m)
{
    enum { Msgcmd_listonline=71, Msgcmd_countonlines=72 };
    enum { Msgcmd_heartbeat = 91, Msgcmd_ack = 95, Msgcmd_auth = 99, Msgcmd_sendmsg=205 };
    enum { Msgcmd_set_devtok=211 };
    enum { Msgcmd_enter_chatroom=215, Msgcmd_exit_chatroom=216 };

    m.rx_.body.clear();

    boost::optional<json::object> jv0 = json::decode<json::object>( m.rx_.buf );
    if (!jv0) {
        return;
    }
    auto& jv = jv0.value();
    auto& body = json::as<json::object>(jv, "body").value();
    int cmd = json::as<int>(jv,"cmd").value_or(0);

    switch (cmd) {
    case Msgcmd_listonline:
        {
            json::array ja;
            BOOST_FOREACH(auto & x, ims.indices_) {
                ja.push_back( std::to_string(*x) );
            }
            m.append_writq(imcprotocol::pack(cmd, ja));
        }
        break;
    case Msgcmd_countonlines:
        {
            auto& lis = boost::get<1>(imc_index_);
            json::object jo;
            jo.emplace("count", lis.size());
            m.append_writq(imcprotocol::pack(cmd, jo));
        }
        break;
    case Msgcmd_heartbeat:
        break;
    case Msgcmd_ack:
        {
            auto msgid = json::as<UInt>(body,"msgid").value();
            std::string sid = json::as<std::string>(body,"sid").value();
            Message_push_server::finish(m.uid_, msgid, sid);
        }
        break;
    case Msgcmd_auth:
        {
            auto & tok = json::as<std::string>(body, "token").value();
            User_auth_server::authrize(response_auth{ IMC_fsm_ptr(&m) }, m.uid_, tok);
        }
        break;
    default: 
        if (cmd >= 99) {
            ;
        }
        break;
    }
}

//inline void iterator_ref_incr(Socket_list<Socket_server>& container, Socket_list<Socket_server>::raw_iterator it)
//{
//    it->u3_incr();
//}
//inline void iterator_ref_decr(Socket_list<Socket_server>& container, Socket_list<Socket_server>::raw_iterator it)
//{
//    if (it->u3_decr()==0) {
//        container.erase(it);
//        //container, mark_alive();
//    }
//}

bool handle_accept(tcp::socket* s)
{
    IMC_fsm_ptr ptr(imc_pool_.construct(std::move(*s)));
    ptr->start();

    //auto& lis = Server::instance();
    //auto p = lis.emplace(std::move(s));
    //if (p.second) {
    //    auto it = p.first;
    //    it->async_recv([it](boost::system::error_code ec, boost::asio::streambuf& buf){
    //            handle_read_g(it, ec, std::move(buf));
    //        });
    //}
    return 1;
}

tcp::endpoint local_endpoint(char const* host, char const* port)
{
    return tcp::endpoint(ip::address::from_string(host), std::stoi(port));
}

void test(int argc, char *const argv[])
{
    boost::asio::io_service io_s;
    Acceptor acceptor(io_s, local_endpoint(argv[1], argv[2]));

    Server server;
    server.async_start(local_endpoint(argv[1], argv[2]));

    tcp::socket socket_(io_s);
    acceptor.async_start(socket_, boost::bind(&handle_accept, &socket_));
}

} // namespace // anonymous

int main(int argc, char *const argv[])
{
    test(argc, argv);
}

