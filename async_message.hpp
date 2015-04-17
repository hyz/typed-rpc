#include <time.h>
#include <type_traits>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <boost/noncopyable.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/intrusive/list.hpp>
//#include <boost/intrusive/unordered_set.hpp>
//#include <boost/functional/hash.hpp>
#include <boost/interprocess/streams/vectorstream.hpp>
#include <boost/interprocess/streams/bufferstream.hpp>
//#include <boost/archive/text_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include "singleton.h"

typedef boost::archive::binary_oarchive oArchive;
typedef boost::archive::binary_iarchive iArchive;
//typedef boost::archive::text_oarchive oArchive;
//typedef boost::archive::text_iarchive iArchive;

namespace msm = boost::msm;
namespace mpl = boost::mpl;
namespace intrusive = boost::intrusive;
namespace ip = boost::asio::ip;

typedef unsigned int UInt;

#if !defined(LOG)
struct _Logger {
    template <typename T> _Logger& operator<<(T&& t) const {
        std::clog << t <<" ";
        return *const_cast<_Logger*>(this);
    }
    _Logger(int line, char const*) {}
    ~_Logger() { std::clog <<"\n"; }
};
#define LOG _Logger(__LINE__,__FILE__)
#endif

inline bool _Success(boost::system::error_code const& ec, int line, char const* filename)
{
    if (ec) {
        LOG << ec << ec.message() <<"#"<< line;
    }
    return (!ec);
}
#define _SUCCESS(ec) _Success(ec, __LINE__,__FILE__)

inline boost::asio::mutable_buffers_1 as_buffer(uint32_t& n)
{
    return boost::asio::mutable_buffers_1(static_cast<void*>(&n), sizeof(uint32_t));
}
inline boost::asio::const_buffers_1 as_buffer(uint32_t const& n)
{
    return boost::asio::const_buffers_1(static_cast<void const*>(&n), sizeof(uint32_t));
}

namespace variadic
{
    template<typename> struct invoke;
    template<template<int...> class seq, int... Indices>
    struct invoke<seq<Indices...>>
    {
        template <typename H,typename T>
            static void call_reply(H&& h, T& t) { h(std::get<Indices>(t)...); }
        template <typename H, typename R, typename T>
            static void call_service(H&& h, R rsp, T& t) { h(rsp, std::get<Indices>(t)...); }
    };
    template<template<int...> class seq>
    struct invoke<seq<>> {
        template <typename H,typename T> static void call_reply(H&&, T&) {}
        template <typename H, typename R, typename T> static void call_service(H&&, R, T&) {}
    };

    template<int... S> struct seq { static constexpr int length = sizeof...(S); };
    template<int N, int... S> struct gens : gens<N-1, N-1, S...> {};
    template<int... S> struct gens<0, S...> { typedef seq<S...> type; };

    namespace detail {

        template<typename Sequence> struct pop_front;
        template<template<typename...> class Sequence, typename T, typename ... Args>
        struct pop_front<Sequence<T, Args...>> {
            typedef Sequence<Args...> type;
        };

        //template <typename Tp, typename... T> struct TVar_each;
        //template <typename Tp> struct TVar_each<Tp> { static void sf() {} };
        //template <typename Tp, typename A, typename... T>
        //struct TVar_each<Tp,A,T...> {
        //    BOOST_STATIC_ASSERT(std::tuple_size<Tp>::value==1+sizeof...(T));
        //    static void sf() {
        //        TVar_each<typename pop_front<Tp>::type,T...>::sf();
        //    }
        //};  

        template <typename Tp, size_t N, size_t I=0>
        struct Tuple_load {
            template <typename Ar> static void sf(Ar& ar, Tp& tp) {
                ar >> std::get<I>(tp);
                Tuple_load<Tp,N,I+1>::sf(ar, tp);
            }
        }; 
        template <typename Tp, size_t N> struct Tuple_load<Tp,N,N> {
            template <typename Ar> static void sf(Ar&, Tp&) {}
        };

        template <typename T, typename A> struct save_helper_ {
            template <typename Ar> //&& std::is_convertible<A,T>::value
            static typename std::enable_if<std::is_arithmetic<T>::value && std::is_arithmetic<A>::value, void>::type
            save(Ar & ar, T a) {
                ar << a; // converted save
            }
        };
        template <typename T> struct save_helper_<T,T> {
            template <typename Ar> static void save(Ar & ar, T const& a) {
                ar << a;
            }
        };
        template <typename Tp, typename... A> struct TVar_save;
        template <typename Tp, typename A0>
        struct TVar_save<Tp,A0> {
            BOOST_STATIC_ASSERT(std::tuple_size<Tp>::value>0);
            typedef typename std::tuple_element<0,Tp>::type T0;
            typedef typename std::decay<A0>::type _A0;
            template <typename Ar>
            static void sf(Ar & ar, _A0 const & a0) {
                save_helper_<T0,_A0>::save(ar, a0);
            }
        };
        template <typename Tp, typename A0, typename... A>
        struct TVar_save<Tp,A0,A...> {
            BOOST_STATIC_ASSERT(std::tuple_size<Tp>::value==1+sizeof...(A));
            template <typename Ar> static void sf(Ar& ar, A0&& a0, A&&... a) {
                TVar_save<Tp,A0>::template sf(ar, a0);
                TVar_save<typename pop_front<Tp>::type,A...>::template sf(ar, std::forward<A>(a)...);
            }
        };

        template <typename...> struct Convertible ;
        template <typename F, typename T>
        struct Convertible<F,T> // : std::is_convertible<F,T>
            : std::conditional< std::is_same<F,T>::value
                , std::true_type
                , typename std::conditional< std::is_arithmetic<T>::value
                    , std::is_convertible<F,T>
                    , std::false_type
                    //, typename std::conditional< std::is_same<T,std::string>::value
                    //    , std::is_same<char,typename std::remove_cv<typename std::remove_pointer<F>::type>>
                    //    , std::false_type
                    //>::type
                >::type
            >::type
        {};
        template <typename F, typename T> struct Convertible<std::pair<F,T>> : Convertible<F,T> {};

        template<typename Fr, typename To>
        struct Tuple_convertible
            : std::conditional<Convertible<typename std::tuple_element<0,Fr>::type,typename std::tuple_element<0,To>::type>::value
                , typename Tuple_convertible<typename pop_front<Fr>::type,typename pop_front<To>::type>::type
                , std::false_type
            >::type
        {};
        template <> struct Tuple_convertible<std::tuple<>,std::tuple<>> : std::true_type {};
        template <typename...T> struct Tuple_convertible<std::tuple<T...>,std::tuple<>> : std::false_type {};
        template <typename...T> struct Tuple_convertible<std::tuple<>,std::tuple<T...>> : std::false_type {};

        template <int,typename...> struct Match_result;
        template <int I,typename Tp, typename First_type,typename Second_type>
        struct Match_result<I,Tp,First_type,Second_type> : std::true_type
        {
            BOOST_STATIC_CONSTANT(int,index=I);
            typedef First_type first_type;
            typedef Second_type second_type;
            typedef Tp from_type;
        };
        template <typename Tp>
        struct Match_result<-1,Tp> : std::false_type
        {
            BOOST_STATIC_CONSTANT(int,index=-1);
            typedef std::tuple<> first_type;
            typedef std::tuple<> second_type;
            typedef Tp from_type;
        };

        template <typename Tp, typename Tab, int N, int I=0> 
        struct Table_index
        {
            typedef typename std::tuple_element<I,Tab>::type Pair_type;
            typedef typename std::tuple_element<0,Pair_type>::type First_type;
            typedef typename std::tuple_element<1,Pair_type>::type Second_type;

            typedef typename std::conditional<Tuple_convertible<Tp,First_type>::value//std::tuple_size<Tp>::value != std::tuple_size<First_type>::value
                    , Match_result<I,Tp,First_type,Second_type>
                    , typename Table_index<Tp, Tab, N, I+1>::result_type
                >::type result_type;
        };
        template <typename Tp, typename Tab, int N> struct Table_index<Tp,Tab,N,N>
        { typedef Match_result<-1,Tp> result_type; };

    } // namespace detail

    template <typename Tab,typename Tp> 
    using Table_index = typename detail::Table_index<Tp,Tab,std::tuple_size<Tab>::value,0>::result_type;

    template<typename Tp, typename Ar, typename... A>
    inline void serialize_save(Ar & ar, A&&... a)
    {
        detail::TVar_save<Tp,A...>::template sf(ar, std::forward<A>(a)...);
    }

    template<typename Ar, typename Tp>
    inline void serialize_load(Ar& ar, Tp & tp)
    {
        detail::Tuple_load<Tp,std::tuple_size<Tp>::value>::template sf(ar, tp);
    }
} // namespace variadic

template <typename D_, typename Request_response>
struct Multic_query_list__ : std::list<Request_response>
{
    Multic_query_list__(boost::asio::io_service& io_s)
        : sock(io_s)
    {}

    template <typename...T>
    void async_do(T&&... t)
    {
        bool empty = this->empty();
        this->emplace(this->end(), std::forward<T>(t)...);

        if (empty) {
            D_::instance()._new_query(this);
        }
    }

    ip::tcp::socket sock;

    struct recvbuf : std::vector<char> {
        uint32_t len = 0;
    };
    recvbuf buf;

    unsigned int reference_count_ = 0;
    intrusive::list_member_hook<> list_hook_;
};

BOOST_STATIC_CONSTANT(int, Nc_default=64);

template <typename Derived, typename Protocol, int Nc=Nc_default>
struct Multic : msm::front::state_machine_def<Multic<Derived,Protocol,Nc>> //, boost::noncopyable
{
    typedef Multic<Derived, Protocol,Nc> This;
    typedef msm::back::state_machine<This> SMac;

    //ip::tcp::resolver resolver;
    struct Ev_request {};
    struct Ev_network_error {};
    struct Ev_connected {};
    struct Ev_resolved {};

    typedef Multic_query_list__<Derived,typename Protocol::request_response> query_list;
    typedef intrusive::list<query_list, intrusive::member_hook<query_list,intrusive::list_member_hook<>, &query_list::list_hook_>> list_type;
    list_type querys;
    typedef boost::intrusive_ptr<query_list> query_ptr;

    ~Multic()
    {
        while (!querys.empty()) {
            auto& x = querys.front();
            querys.pop_front();
            _destroy(&x);
        }
    }
    Multic(boost::asio::io_service& io_s, std::string h, unsigned short port)
        : host(std::move(h))
        , endpx(ip::tcp::v4(), port)
        , sock_(io_s)
        , timer_(io_s)
    {}
    Multic(boost::asio::io_service& io_s, ip::address ipa, unsigned short port)
        : endpx(ipa, port)
        , sock_(io_s)
        , timer_(io_s)
    {}
    // void reset() { sockets.clear(); }

    boost::intrusive_ptr<query_list> make_query()
    {
        return boost::intrusive_ptr<query_list>( qa_pool_.construct(get_io_service()) );
    }

    template <typename... T>
    boost::intrusive_ptr<query_list> async_do(T&&... t)
    {
        boost::intrusive_ptr<query_list> q = make_query(); //qa_pool_.construct(get_io_service());
        q->async_do(std::forward<T>(t)...);
        return q;
    }

    boost::asio::io_service& get_io_service() { return timer_.get_io_service(); }

    struct Wait_request : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    //struct Wait_response : public msm::front::state<> {};
    struct Wait_connect : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_resolve : msm::front::interrupt_state<Ev_resolved> {
        template <class Ev, class M> void on_entry(Ev const&, M& m)
        {
            //LOG << "wait resolv";
            m.timer_.expires_from_now(boost::posix_time::seconds(2));
            m.timer_.async_wait([&m](boost::system::error_code){
                        m.stop();
                        m.start();
                        // m.process_event(Ev_resolved());
                        if (!m.querys.empty()) {
                            m.process_event(Ev_request());
                        }
                    });
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Resolved : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M& m)
        {
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct _Switch_next : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };

    struct Make_connection {
        template <class Ev, class M, class S, class T>
        void operator()(Ev const& ev, M& m, S&, T&) {
            BOOST_ASSERT(!m.sock_.is_open());
            //! m.sock_.open(ip::tcp::v4());
            m.sock_.async_connect(m.endpx, Handle_connect{&m});
        }
    };
    struct Write_sockets {
        template <class Ev, class M, class S, class T>
        void operator()(Ev const& ev, M& m, S&, T&) {
            // BOOST_ASSERT(!m.sockets.empty());
            if (m.sockets.empty()) {
                boost::system::error_code ec;
                m.sock_.connect(m.endpx, ec);
                if (!_SUCCESS(ec)) {
                    m.sock_.close(ec); // BOOST_ASSERT(!m.sock_.is_open());
                    m.process_event(Ev_network_error());
                    return;
                }
                m.sockets.emplace(m.sockets.end(), std::move(m.sock_));
                m.n_sock++;
                //LOG << "socket:connect" << m.n_sock;
            }
            This::_send_nxt(m);
        }
    };

    struct Handle_connect {
        SMac* m; // int native_handle = 0; // sockets sk_ptr;
        void operator()(boost::system::error_code ec)
        {
            if (!_SUCCESS(ec)) {
                BOOST_ASSERT(!m->sock_.is_open());
                m->process_event(Ev_network_error());
                return;
            }
            //LOG << "connected" << m->sock_.native_handle();
            m->sockets.emplace(m->sockets.end(), std::move(m->sock_));
            m->n_sock++;
            m->process_event(Ev_connected());
            BOOST_ASSERT(!m->sock_.is_open());
        }
    };

    struct Handle_write {
        query_ptr ptr;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            if (!_SUCCESS(ec)) {
                return This::_handle_write_error(ec, ptr);
            }
            boost::asio::async_read(ptr->sock, as_buffer(ptr->buf.len), Handle_read_size{ptr});
        }
    };
    struct Handle_read_size {
        query_ptr ptr; // query_iterator it;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            if (!_SUCCESS(ec)) {
                return This::_handle_write_error(ec, ptr);
            }
            ptr->buf.resize(ntohl(ptr->buf.len));
            boost::asio::async_read(ptr->sock, boost::asio::buffer(ptr->buf), Handle_read_content{ptr});
        }
    };
    struct Handle_read_content {
        query_ptr ptr; // query_iterator it;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            if (!_SUCCESS(ec)) {
                return This::_handle_write_error(ec, ptr);
            }
            BOOST_ASSERT(!ptr->empty());
            BOOST_ASSERT(!ptr->list_hook_.is_linked());
            BOOST_ASSERT(ptr->buf.size() == bytes_transferred);

            auto& m = Derived::instance();
            m.sockets.push_back(std::move(ptr->sock));
            BOOST_ASSERT (!ptr->sock.is_open());

            if (!ptr->buf.empty()) {
                ptr->front()(ptr->buf.data(), ptr->buf.size()); //::: callback handler
            }

            ptr->pop_front();
            if (!ptr->empty()) {
                m.querys.push_back(*ptr);
                // m.querys.erase( m.querys.iterator_to(*ptr) );
            }

            This::_send_nxt(m);
        }
    };

    struct Under_limit {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S&, T&) const {
            return m.n_sock < Nc;
        }
    };
    struct Sockets_not_empty {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts) const {
            return m.n_sock < Nc; //return !m.sockets.empty();
        }
    };
    struct Sockets_is_empty {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts) const {
            return m.sockets.empty();
        }
    };
    struct Has_request_and_Under_limit {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts) const {
            return !m.querys.empty() ; //&& !m.sockets.empty();
        }
    };

    typedef mpl::vector<Wait_request,Resolved> initial_state;
    template <typename... T> using Row = msm::front::Row<T...> ;
    typedef msm::front::none None;

    struct transition_table : mpl::vector<
        Row< Wait_request , Ev_request       , Wait_connect , Make_connection  , Under_limit          >
      , Row< Wait_request , Ev_request       , Wait_request , Write_sockets    , Sockets_not_empty    >
      , Row< Wait_connect , boost::any       , Wait_request , None             , None                 >
      , Row< Wait_connect , Ev_connected     , _Switch_next , Write_sockets    , None                 >
      , Row< _Switch_next , None             , Wait_request , None             , None                 >
      , Row< _Switch_next , None             , Wait_connect , Make_connection  , Has_request_and_Under_limit >
        /// / // / // / ///
      , Row< Wait_resolve , Ev_resolved      , Resolved     , None             , None                 >
      , Row< Resolved     , Ev_network_error , Wait_resolve , None             , Sockets_is_empty     >
    > { };

    template <class Ev, class M>
    void no_transition(Ev const& ev, M& fsm, int state)
    {}

    template <class Ev, class M> void on_entry(Ev const& ev, M& m)
    {}
    template <class Ev, class M> void on_exit(Ev const&, M&) {}

    void _close(ip::tcp::socket& sock)
    {
        //LOG << "is_open" << sock.is_open();
        boost::system::error_code ec;
        sock.close(ec);
        n_sock--;
    }

    static void _handle_write_error(boost::system::error_code ec, query_ptr ptr)
    {
        auto& m = Derived::instance();
        m._close(ptr->sock);
        m.querys.push_back(*ptr); // m.querys.splice(m.querys.begin(), m.querys, m.querys.iterator_to(*ptr));
        m.process_event(Ev_network_error());
    }
    static void _send_nxt(SMac& m)
    {
        while (!m.sockets.empty() && !m.querys.empty()) {
            auto it = m.querys.begin();
            BOOST_ASSERT(!it->empty());
            BOOST_ASSERT(!it->sock.is_open());

            query_ptr ptr(it.operator->());
            m.querys.pop_front(); //.erase(it); // m.querys.splice(m.querys.end(), m.querys, it);

            ptr->sock = std::move( m.sockets.back() );
            m.sockets.pop_back();

            auto& q = ptr->front();
            boost::asio::async_write(ptr->sock, q.const_buffer(), Handle_write{ptr});
        }
    }

    void _new_query(query_list* p)
    {
        BOOST_ASSERT(!p->list_hook_.is_linked());
        this->querys.push_back(*p); //(boost::intrusive_ptr<query_list>(p));

        Derived::instance().process_event(Ev_request{});
    }

    void _destroy(query_list* p)
    {
        BOOST_ASSERT (!p->list_hook_.is_linked());
        qa_pool_.destroy(p);
    }

    ip::tcp::endpoint endpx;
    std::string host;

    std::vector<ip::tcp::socket> sockets;
    ip::tcp::socket sock_;

    unsigned short n_req = 0;
    unsigned short n_sock = 0;

    boost::asio::deadline_timer timer_;
    boost::object_pool<query_list> qa_pool_;
    // typedef boost::singleton_pool<int,sizeof(query_list)> s_pool_t;
};

template <typename Derived, typename RR>
inline void intrusive_ptr_add_ref(Multic_query_list__<Derived,RR> * p)
{
    ++p->reference_count_;
}
template <typename Derived, typename RR>
inline void intrusive_ptr_release(Multic_query_list__<Derived,RR> * p)
{
    if (--p->reference_count_ == 0 && !p->list_hook_.is_linked()) {
        Derived::instance()._destroy(p);
    }
}

/// / // / // / /// /// / // / // / /// /// / // / // / /// /// / // / // / ///

namespace Message {
    template <typename...T> using Request = std::tuple<T...>;
    template <typename...T> using Response = std::tuple<T...>;
    template <typename...T> using Pair = std::tuple<T...>;
    template <typename...T> using Table = std::tuple<T...>;

    template <typename T>
    struct tag {
        template<class Archive> inline void serialize(Archive&, const unsigned int) {}
    };
} // namespace Message

template <class Table>
struct Message_client_side
{
    typedef Message_client_side This;

    struct request_buffer : std::vector<char> {
        uint32_t olen = 0;
        typedef std::array<boost::asio::const_buffer,2> const_buffers_t;
        const_buffers_t const_buffer() const
        {
            return const_buffers_t{ as_buffer(olen), boost::asio::buffer(*this) };
        }
    };

    struct request_response
                : request_buffer
                , std::function<void(char const*,size_t)>
    {
        template <typename Fn, typename... A>
        request_response(Fn&& fn, A&& ... a)
        {
            typedef variadic::Table_index<Table, std::tuple<typename std::decay<A>::type...>> Tp;
            _init_req(static_cast<Tp*>(0), std::forward<A>(a)...);
            _init_rsp(static_cast<Tp*>(0), std::forward<Fn>(fn));
        }

        template <int Ix, typename Tpx>
        static bool unpack_1(Tpx& rsp, char const* buf, size_t len)
        {
            typedef boost::interprocess::basic_bufferbuf<char> bufferbuf;
            bufferbuf sbuf(const_cast<char*>(buf), len);
            iArchive ia(sbuf, boost::archive::no_header);
            uint8_t ix;
            ia & ix;
            BOOST_ASSERT(ix == Ix);
            if (Ix != ix) {
                return 0;
            }
            variadic::serialize_load(ia, rsp);
            return 1;
        }

        template <typename Tpx, typename Fn>
        void _init_rsp(Tpx*, Fn&& fn)
        {
            std::function<void(char const*,size_t)>& base = *this;
            base = [fn](char const* buf, size_t len) mutable {
                typedef typename Tpx::second_type Tuple;
                Tuple rsp;
                if (request_response::unpack_1<Tpx::index>(rsp, buf,len)) {
                    variadic::invoke<typename variadic::gens<std::tuple_size<Tuple>::value>::type>::call_reply(fn, rsp);
                }
            };
        }
        template <typename Tpx, typename... A>
        void _init_req(Tpx*, A&& ... a)
        {
            typedef boost::interprocess::basic_vectorbuf<std::vector<char>> vectorbuf;
            vectorbuf vecbuf;
            uint8_t ix = Tpx::index;
            {
                oArchive oa(vecbuf, boost::archive::no_header);
                oa << ix;
                variadic::serialize_save<typename Tpx::first_type>(oa, std::forward<A>(a)...);
            }
            std::vector<char>& v = *this;
            vecbuf.swap_vector(v);
            this->olen = ntohl(v.size());
        }
    };
};

template <typename Agent, typename Message_table>
    using agent_def = msm::back::state_machine<Multic<Agent,Message_client_side<Message_table>>>;

template<typename Handle, typename Table>
struct Message_server_side
{   
    template <int Ix, typename Tuple>
    struct reply_pack
    {
        boost::intrusive_ptr<Handle> ptr; // // std::vector<char>* replybuf;
        template <typename... T>
        void operator()(T&&... t) const
        {
            //BOOST_STATIC_ASSERT(std::is_same<Tuple,std::tuple<typename std::decay<T>::type...>>::value);
            typedef boost::interprocess::basic_vectorbuf<std::vector<char>> vectorbuf;
            vectorbuf vecbuf; {
                uint8_t ix = Ix;
                oArchive oa(vecbuf, boost::archive::no_header);
                oa & ix;
                variadic::serialize_save<Tuple>(oa, std::forward<T>(t)...);
            }
            std::vector<char> sbuf;
            vecbuf.swap_vector(sbuf);//(*replybuf);
            ptr->reply( std::move(sbuf) );
        }
    };

    template<int N, int...Indices> struct Lup : Lup<N-1, N-1, Indices...> {};
    template<int...Indices> struct Lup<0, Indices...>
    {
        template <int Ix> static void spf_(iArchive& ar, boost::intrusive_ptr<Handle>& h/*, std::vector<char>& replybuf*/) {
            typedef typename std::tuple_element<Ix,Table>::type T2;
            typedef typename std::tuple_element<0,T2>::type Tuple;
            Tuple tp;
            variadic::serialize_load(ar, tp);
            reply_pack<Ix,typename std::tuple_element<1,T2>::type> rsp{ h };
            variadic::invoke<typename variadic::gens<std::tuple_size<Tuple>::value>::type>::call_service(*h, rsp, tp);
        }
        static void dispa_(iArchive& ar, boost::intrusive_ptr<Handle>& h/*, std::vector<char>& replybuf*/) {
            typedef void(*FType)(iArchive&, boost::intrusive_ptr<Handle>&);
            static FType lookup[sizeof...(Indices)] = { &Lup::spf_<Indices>... };
            uint8_t ix; {
                ar >> ix;
            }
            if (ix < sizeof...(Indices)) {
                (*lookup[ix])(ar, h);
            }
        }
    };

    static void dispatch(std::vector<char>& buf, boost::intrusive_ptr<Handle> h) //const
    {
        typedef boost::interprocess::basic_bufferbuf<char> bufferbuf;
        bufferbuf sbuf(buf.data(), buf.size());
        iArchive ia(sbuf, boost::archive::no_header);
        Lup<std::tuple_size<Table>::value>::dispa_(ia, h);
    }
};

////////////////////////////////////////////////////////////////////////////////////

template <typename D, typename B>
static constexpr D* downcast(B* x)
{
    BOOST_STATIC_ASSERT(std::is_base_of<B,D>::value);
    return static_cast<D*>(x);
};
template <typename D, typename B> static constexpr D* downcast(B const* x) { return downcast<D>(const_cast<B*>(x)); };

template <typename service> struct server; //struct EmptyS_ {};

template <typename Handle, typename Table/*, typename Extend=EmptyS_*/>
struct service_ : msm::front::state_machine_def<service_<Handle,Table>>
                , boost::intrusive::list_base_hook<>
{
    // typedef service_ This;
    typedef boost::intrusive_ptr<Handle> pointer;

    pointer cast_this() const { return pointer( downcast<Handle>(this) ); }

    unsigned int reference_count_ = 0;
    time_t tpa_ = 0;
    ip::tcp::socket socket;

    struct message : std::vector<char> {
        uint32_t len = 0;
    } requestbuf, replybuf;

    service_(ip::tcp::socket* sk)
        : socket(std::move(*sk))
    { }

    template <typename Ev> void fire_event(Ev&& ev) {
        //LOG << typeid(Ev).name();
        downcast<Handle>(this)->process_event(ev);
    }

    struct Ev_close    {};
    struct Ev_read     {};
    struct Ev_writeall {};
    struct Ev_response {};

    struct rsphandle_ignore {
        template <typename...T> void operator()(T&&...) const {}
    };

    struct handle_read {
        pointer ptr;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            auto& m = *ptr;
            if (!_SUCCESS(ec)) {
                m.fire_event(Ev_close());
                return;
            }
            m.fire_event(Ev_read());
        }
    };
    struct handle_write {
        pointer ptr;
        void operator()(boost::system::error_code ec, size_t bytes_transferred)
        {
            auto& m = *ptr;
            if (!_SUCCESS(ec)) {
                m.fire_event(Ev_close());
                return;
            }
            m.replybuf.clear();
            m.fire_event(Ev_writeall());
        }
    };

    struct Wait_header : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {
            //LOG << typeid(Ev).name() << "Wait_header";
            server<Handle>::instance()._mark_live( *downcast<Handle>(&m) );
            m.requestbuf.len = 0;
            m.requestbuf.clear();
            boost::asio::async_read(m.socket
                    , as_buffer(m.requestbuf.len) , handle_read{ m.cast_this() });
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_body : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const& ev, M& m)
        {
            //LOG << typeid(Ev).name() << "Wait_body";
            m.requestbuf.len = ntohl(m.requestbuf.len);
            m.requestbuf.resize(m.requestbuf.len);
            boost::asio::async_read(m.socket
                    , boost::asio::buffer(m.requestbuf)
                    , handle_read{ m.cast_this() });
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) { }
    };
    struct Wait_response : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&) {
            //LOG << typeid(Ev).name() << "Wait_response";
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Wait_write : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M& m)
        {
            m.replybuf.len = htonl(m.replybuf.size());
            boost::array<boost::asio::const_buffer,2> bufs{ as_buffer(m.replybuf.len), boost::asio::buffer(m.replybuf) };
            boost::asio::async_write(m.socket, bufs, handle_write{ m.cast_this() });
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Process_message : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M& m) {
            //LOG << typeid(Ev).name() << "Process_message";
            BOOST_ASSERT(m.replybuf.empty());
            Message_server_side<Handle,Table>::dispatch(m.requestbuf, m.cast_this()); //::: server-side
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Running : public msm::front::state<> {
        template <class Ev, class M> void on_entry(Ev const&, M&) {}
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Closing : msm::front::interrupt_state<Ev_writeall> {
        template <class Ev, class M> void on_entry(Ev const&, M&)
        {} // if (m.writq.empty()) { m.process_event(Ev_writeall()); }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };
    struct Closed : msm::front::terminate_state<> {
        template <class Ev, class M> void on_entry(Ev const&, M& m) {
            //LOG << typeid(Ev).name() << "Closed";
            boost::system::error_code ec;
            m.socket.close(ec);
        }
        template <class Ev, class M> void on_exit(Ev const&, M&) {}
    };

    struct AlwaysTrue {
        template <class Ev, class M, class S, class T> bool operator()(Ev const& ev, M& m, S& ss, T& ts) { return 1; }
    };
    struct Has_response {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S& ss, T& ts) { return (!m.replybuf.empty()); }
    };
    struct Is_request {
        template <class Ev, class M, class S, class T>
        bool operator()(Ev const& ev, M& m, S&, T&) { return 1; }
    };

    //typedef Wait_header initial_state;
    typedef mpl::vector<Wait_header,Running> initial_state;

    template <typename... T> using Row = msm::front::Row<T...> ;
    typedef msm::front::none None;

    struct transition_table : mpl::vector<
        Row< Wait_header     , Ev_read     , Wait_body       , None  , None         >
      , Row< Wait_body       , Ev_read     , Process_message , None  , None         >
      , Row< Wait_response   , Ev_response , Wait_write      , None  , None         >
      , Row< Wait_response   , None        , Wait_write      , None  , Has_response >
      , Row< Wait_write      , Ev_writeall , Wait_header     , None  , None         >
      , Row< Process_message , None        , Wait_header     , None  , None         >
      , Row< Process_message , None        , Wait_response   , None  , Is_request   >
        /// ///
      , Row< Running       , Ev_close    , Closed        , None            , None           >
      , Row< Running       , Ev_close    , Closing       , None            , Has_response   >
      , Row< Closing       , Ev_writeall , Closed        , None            , None           >
    > {};

    template <class Ev, class M>
    void no_transition(Ev const& ev, M& fsm, int state) {
        //LOG << typeid(Ev).name() << state;
    }

    template <class Ev, class M> void on_entry(Ev const& ev, M& m) {
        //LOG << typeid(Ev).name();
    }
    template <class Ev, class M> void on_exit(Ev const&, M&) {
        //LOG << typeid(Ev).name();
    }

    void reply(std::vector<char>&& buf) {
        static_cast<std::vector<char>&>(replybuf) = std::move(buf);
    }

//public:
//    BOOST_STATIC_CONSTANT(int, Bucket_size=1);
//    typedef boost::hash<UInt> hash_key_func;
//    struct hash_key_equal {
//        bool operator()(UInt u, This const& rhs) const { return 0; }
//    };
//    friend bool operator==(const This &l, const This &r) {  return 0;   }
//    friend std::size_t hash_value(const This &v) {  return 0; }
}; ///////////////////////// service_

template <typename Service, typename Table>
    using service_def = msm::back::state_machine<service_<Service,Table>>;

template <typename service>
struct server : boost::intrusive::list<service>, singleton<server<service>>
{
    typedef server This;
    typedef boost::intrusive_ptr<service> pointer;

    template <typename... T>
    static pointer construct(T&&... t) {
        auto& self = This::instance();
        pointer p( self.pool_.construct(std::forward<T>(t)...) );
        self.push_back(*p);
        self._mark_live(*p);
        return p;
    }

//private: /////////////////////
    void _mark_live(service& m) {
        m.tpa_ = time(0);
        this->splice(this->end(), *this, this->iterator_to(m));
    }

    void _destroy(service* p) {
        if (p->is_linked()) { //(p->list_hook_0_.is_linked())
            this->erase( this->iterator_to(*p) );
        }
        this->pool_.destroy(p);
    }

//    //void check()
//    //{
//    //    auto& lis = This::instance().lis_;
//    //    if (lis.empty()) {
//    //        return;
//    //    }
//    //    enum { Ts=60*6 };
//    //    time_t tpc = time(0);
//
//    //    boost::intrusive::list<IMC_service>::iterator const b = lis.begin();
//    //    boost::intrusive::list<IMC_service>::iterator it;
//    //    while ( (it = lis.begin()) != b) {
//    //        if (it->tplive_ + Ts < tpc) {
//    //            lis.splice(lis.end(), lis, it);
//    //            LOG << "expire" << it->connection_info();
//    //            it->close();
//    //        }
//    //    }
//    //}

    boost::object_pool<service> pool_;
};

template <typename Service, typename Table>
inline void intrusive_ptr_add_ref (service_def<Service,Table>* p)
{
    ++p->reference_count_;
}
template <typename Service, typename Table>
inline void intrusive_ptr_release(service_def<Service,Table>* p)
{
    if (--p->reference_count_ == 0) {
        server<Service>::instance()._destroy( downcast<Service>(p) );
    }
}

struct Acceptor : boost::noncopyable
{
    ip::tcp::socket socket;
    ip::tcp::acceptor acceptor_;

    Acceptor(boost::asio::io_service& io_s, ip::tcp::endpoint endpx)
        : socket(io_s)
        , acceptor_(io_s, endpx, true)
    {
        // LOG << endpx;

        //acceptor_.open(endpx.protocol());
        acceptor_.set_option(ip::tcp::acceptor::reuse_address(true));
        //acceptor_.bind(endpx);
        acceptor_.listen();
    }

    template <typename Fn>
    void async_accept(Fn fn)
    {
        boost::system::error_code ec;
        socket.close(ec);
        //socket.open(ip::tcp::v4());
        acceptor_.async_accept(socket, fn);
    }
};

