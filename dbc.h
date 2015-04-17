#ifndef _DBC_H_
#define _DBC_H_

#include "config.hpp"
#include <stdexcept>
#include <string>
#include <list>
#include <vector>
#include <sstream>
#include <iostream>

#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/asio/buffer.hpp>

#include <mysql/mysql.h>
#include <hiredis/hiredis.h>

#include "myerror.h"
#include "log.h"

namespace sql { // namespace

struct error : ::myerror {};
struct error_connect : error {};
struct error_query : error {};
struct error_exec : error {};

struct dbc;
struct datas;

typedef MYSQL* native_handle_type;

struct dbc : boost::noncopyable
{
    struct _c_type
    {
        native_handle_type c_;
        int using_;
        _c_type(native_handle_type c) { using_ = 0; c_ = c; }
    };

    typedef std::list<_c_type>::iterator iterator;

    struct config
    {
        std::string host;
        int port;
        std::string user, pwd;
        std::string db;

        config(boost::property_tree::ptree const& ini);
    };

    struct connection
    {
        ~connection() { --i_->using_; }
        connection(const connection& rhs) : i_(rhs.i_) { ++i_->using_; }
        connection& operator=(const connection& rhs);

        native_handle_type native_handle() const { return i_->c_; }

    private:
        explicit connection(iterator i) : i_(i) { ++i_->using_; }
        iterator i_;
        friend struct dbc;
    };

    connection connect();

    bool empty() const { return cs_.empty(); }

    dbc(const config& cfg)
        : cfg_(cfg)
    {}

    // dbc() {}

    ~dbc();

    static dbc& instance(dbc::config const* c = 0);
    static void init(dbc::config const& cfg);

private:
    config cfg_; // std::string host_; int port_; std::string user_, pwd_, db_;

    boost::mutex mutex_;
    std::list<_c_type> cs_;
};

struct datas;

struct datas_row_type
{
    operator bool() const { return bool(row_); }

    const char* at(size_t idx) const;
    const char* at(size_t idx, const char* defa) const
    {
        char const* p = at(idx);
        return p ? p : defa;
    }

    //const char* at(unsigned int idx) const
    //    { return const_cast<datas_row_type*>(this)->at(idx); }
    //const char* at(unsigned int idx, const char *defa) const
    //    { return const_cast<datas_row_type*>(this)->at(idx,defa); }

    const char* operator[](size_t idx) { return at(idx); }
    const char* operator[](size_t idx) const { return at(idx); }

private:
    MYSQL_ROW row_;
    datas* datas_;

    datas_row_type(MYSQL_ROW r, datas& ds)
    {
        row_ = r;
        datas_ = &ds;
    }

    friend struct datas;
};

struct datas : boost::noncopyable
{
    typedef datas_row_type row_type;

    // struct iterator;

    template <typename Sql>
    datas(const Sql& sql, const dbc::connection& c)
        : dbc_(c)
    {
        result_ = 0;
        query(sql);
        n_field_ = mysql_num_fields(result_);
    }

    template <typename Sql>
    explicit datas(const Sql& sql)
        : dbc_(dbc::instance().connect())
    {
        result_ = 0;
        query(sql);
        n_field_ = mysql_num_fields(result_);
    }

    ~datas()
    {
        if (result_) {
            mysql_free_result(result_);
        }
    }

    row_type next()
    {
        return row_type(mysql_fetch_row(result_), *this);
    }

    size_t count() const { return result_ ? mysql_num_rows(result_) : 0; }
    size_t count_fields() const { return n_field_; }//{ return mysql_num_fields(result_); }

private:
    dbc::connection dbc_;
    MYSQL_RES* result_;
    size_t n_field_;
    // std::vector<std::string> fields_;
    // unsigned int _index(const std::string& field);

    void query(const std::string& sql);
    void query(const boost::format& sql) { query(sql.str()); }
};

inline const char* datas_row_type::at(size_t idx) const
{
    if (idx >= size_t(datas_->count_fields()))
        BOOST_THROW_EXCEPTION( sql::error() );
    return row_[idx];
}

int exec(boost::system::error_code & ec, const std::string& sql, const dbc::connection& c);

template <typename Sql>
int exec(boost::system::error_code & ec, const Sql& sql)
{
    return exec(ec, sql, dbc::instance().connect());
}

template <typename Sql>
int exec(const Sql& sql)
{
    boost::system::error_code ec;
    int x = exec(ec, sql, dbc::instance().connect());
    if (ec) {
        BOOST_THROW_EXCEPTION( sql::error() );
    }
    return x;
}

inline int exec(boost::system::error_code & ec, const boost::format& sql, const dbc::connection& c)
{
    return exec(ec, sql.str(), c);
}

// template <typename Sql> inline void exec(const Sql& sql)
// {
//     exec(sql, dbc::instance().connect());
// }

std::string db_escape(const std::string& param, const dbc::connection& c);

inline std::string db_escape(const boost::format& param, const dbc::connection& c)
{
    return db_escape(param.str(), c);
}

template <typename Sql_param>
inline std::string db_escape(const Sql_param& param)
{
    return db_escape(param, dbc::instance().connect());
}

inline std::string escape(std::string const & s) { return db_escape(s); }

//struct initializer : boost::noncopyable
//{
//    initializer (const boost::property_tree::ptree & ini);
//    ~initializer();
//};

} // namespace

//template <typename Tab>
//int max_index(int x, char const *tab=0)
//{
//    static int idx_ = 0;
//    if (idx_ == 0)
//    {
//        idx_ = 1;
//
//        sql::datas datas(std::string("SELECT MAX(id) FROM ") + (tab ? tab : Tab::db_table_name()));
//        if (sql::datas::row_type row = datas.next())
//        {
//            if (row[0])
//                idx_ = std::atoi(row[0]) + 1;
//        }
//    }
//    int ret = idx_;
//    idx_ += x;
//    return ret;
//}

namespace boost {
    template <>
    inline std::string lexical_cast(boost::asio::const_buffers_1 const& b)
    {
        return std::string(boost::asio::buffer_cast<char const*>(b), boost::asio::buffer_size(b));
    }
}
namespace redis {

void init(boost::property_tree::ptree const& ini);

typedef boost::shared_ptr<redisContext> context_ptr;
typedef std::pair<std::string, int> endpoint;
typedef boost::shared_ptr<redisReply> reply;

inline std::ostream& operator<<(std::ostream& out, endpoint const& ep)
{
    return out << ep.first<<","<< ep.second;
}

struct context_helper : boost::noncopyable
{
    redis::reply reply();

    ~context_helper();
    context_helper(endpoint const& ep);

protected:
    context_ptr context_ptr_;
    endpoint endp_;
    int n_reply_;
    //auto_cpu_timer_helper cput_; //AUTO_CPU_TIMER
};

struct context : context_helper //, private boost::noncopyable
{
    template <typename... Args>
        context & append(Args const&... a);

    ~context();
    explicit context(); //(context_helper ctx) : context_helper(ctx) {  }
    explicit context(std::string const& h, unsigned short p); // : context_helper(ctx) {  }

private:
    std::vector<char const*> argv_;
    std::vector<size_t> argv_len_;
    std::list<std::string> tmps_;

    template <typename T> void app(T const& x)
    {
        this->app2(x);
    }

    template <typename T, typename... Args> void app(T const& x, Args const&... a)
    {
        this->app2(x);
        this->app(a...);
    }

    void app2(char const * s)
    {
        argv_.push_back(s);
        argv_len_.push_back(strlen(s));
    }
    void app2(std::string const & s)
    {
        argv_.push_back(s.data());
        argv_len_.push_back(s.size());
    }
    template <typename T> void app2(T const & x)
    {
        tmps_.push_back( boost::lexical_cast<std::string>(x) );
        this->app2(tmps_.back());
    }
};

template <typename... Args>
context & context::append(Args const&... a)
{
    this->app(a...);

    redisAppendCommandArgv(context_ptr_.get(), argv_.size(), &argv_[0], &argv_len_[0]);
    ++n_reply_;

    LOG << argv_ << "redis" << n_reply_;
    argv_.clear();
    argv_len_.clear();
    tmps_.clear();
    return *this;
}

template <typename ...Args>
redis::reply command(Args const&... a)
{
    redis::context ctx; // (redis::context::make());
    ctx.append(a...);
    return ctx.reply();
}

template <typename OIter>
void keys(std::string const & kp, OIter it)
{
    auto reply = redis::command("KEYS", kp);
    if (!reply || reply->type != REDIS_REPLY_ARRAY)
        return;

    for (unsigned int i = 0; i < reply->elements; i++)
        *it++ = (reply->element[i]->str);
}

} // namespace redis

//#if 0
//#include "mongo/client/dbclient.h"
//
//namespace MyMongo
//{
//#define DEFA_MONGO_HOST  "192.168.1.57"
//#define DEFA_MONGO_PORT 27017
//
//    struct error : ::myerror {};
//    struct error_init : error {};
//    struct error_connect : error {};
//    struct error_query : error {};
//
//    class mongo_conn_mgr 
//    {
//        public:
//            enum { MASTER = 1, SLAVE };
//            typedef boost::shared_ptr<mongo::DBClientConnection> Mongo_Conn_Ptr;
//            typedef std::pair<Mongo_Conn_Ptr ,bool> MongoConnect;
//            typedef std::vector<MongoConnect>::iterator iterator;
//
//            static void init(boost::property_tree::ptree const& ini);
//            Mongo_Conn_Ptr get_connect( int type = MASTER );
//            static mongo_conn_mgr& inst();
//
//        private:
//            std::vector< MongoConnect > m_conn_pool_, s_conn_pool_;
//            bool create_connect( int type = MASTER );
//
//            static std::string mongo_host_;
//            static unsigned short mongo_port_;
//
//    };
//
//    void mongo_insert( const std::string& db_table, const mongo::BSONObj& in );
//    boost::shared_ptr<mongo::DBClientCursor> mongo_select(const std::string& db_table, const mongo::Query& in );
//}
//#endif // mongodb

#endif // _DBC_H_

