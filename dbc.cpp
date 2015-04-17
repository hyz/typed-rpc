#include <map>
// #include <boost/container/map.hpp>
#include <boost/assert.hpp>
#include <boost/make_shared.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "myerror.h"
#include "log.h"
#include "dbc.h"

using namespace std;
using namespace boost;

ostream& operator<< (ostream& out, const sql::dbc::config& cfg)
{
    return out << boost::format("Database %s:%d %s:%s %s") % cfg.host % cfg.port % cfg.user % cfg.pwd % cfg.db;
}

// static std::string mysql_host_ = "127.0.0.1"; static unsigned short mysql_port_ = 3306;

namespace sql {

// void dbc::init(const property_tree::ptree& ini)
void dbc::init(dbc::config const& cfg) //(const property_tree::ptree& ini)
{
    dbc::instance(&cfg);
}

dbc::config::config(property_tree::ptree const& ini)
{
    this->host = ini.get<std::string>("host", "127.0.0.1");
    this->port = ini.get<int>("port", 3306);
    this->user = ini.get<std::string>("user");
    this->pwd  = ini.get<std::string>("password");
    this->db   = ini.get<std::string>("db");

    LOG << *this;
}

dbc::~dbc()
{
    // boost::unique_lock<boost::mutex> scoped_lock(mutex_);
    for (iterator i = cs_.begin(); i != cs_.end(); ++i) {
        if (i->using_ == 0)
            mysql_close(i->c_);
    }
}

dbc& dbc::instance(dbc::config const* c)
{
    static dbc db_(*c);
    return db_;
}

dbc::connection dbc::connect()
{
    boost::unique_lock<boost::mutex> scoped_lock(mutex_);

    dbc::iterator i = cs_.begin();
    while (i != cs_.end())
    {
        if (i->using_ == 0)
        {
            if (mysql_ping(i->c_) != 0)
            {
                LOG << "MySQL ping " << mysql_errno(i->c_) << mysql_error(i->c_);
                mysql_close(i->c_);
                i = cs_.erase(i);
                continue;
            }
            return connection(i);
        }
        ++i;
    }

    AUTO_CPU_TIMER("sql:connect");
    LOG << "mysql_init " << cs_.size();

    native_handle_type c = mysql_init(NULL);
    if (!c)
    {
        LOG << "MySQL init "<< mysql_errno(c) << mysql_error(c);
        // THROW_EX(EN_DBConnect_Fail);
        BOOST_THROW_EXCEPTION( sql::error_connect() );
    }
    if (!mysql_real_connect(c
                , cfg_.host.c_str(), cfg_.user.c_str(), cfg_.pwd.c_str()
                , cfg_.db.c_str(), cfg_.port, NULL, 0))
    {
        LOG << "MySQL real connect "<< mysql_errno(c) << mysql_error(c);
        // THROW_EX(EN_DBConnect_Fail);
        BOOST_THROW_EXCEPTION( sql::error_connect() );
    }

    if (mysql_set_character_set(c, "utf8mb4"))
    {
        LOG << "MySQL character "<< mysql_errno(c) << mysql_error(c);
    }

    cs_.push_back(_c_type(c));
    return connection(--cs_.end());
}

dbc::connection& dbc::connection::operator=(const dbc::connection& rhs)
{
    if (this != &rhs)
    {
        --i_->using_;
        i_ = rhs.i_;
        ++i_->using_;
    }
    return *this;
}

void datas::query(const string& sql)
{
    BOOST_ASSERT(!result_);
    AUTO_CPU_TIMER("sql:query");

    native_handle_type c = dbc_.native_handle();

    LOG << sql;
    if (0 != mysql_query(c, sql.c_str()))
    {
        LOG << "mysql_query "<< mysql_errno(c) << mysql_error(c);
        // THROW_EX(EN_SQL_Fail);
        BOOST_THROW_EXCEPTION( sql::error_query() );
    }

    result_ = mysql_store_result(c);
}

int exec(boost::system::error_code & ec, const std::string& sql, const dbc::connection& wc)
{
    AUTO_CPU_TIMER("sql:exec");
    LOG << sql;

    native_handle_type c = wc.native_handle();
    if (mysql_query(c, sql.c_str()) != 0)
    {
        LOG << "MySQL query "<< mysql_errno(c) << mysql_error(c);
        BOOST_THROW_EXCEPTION( sql::error_query() );
        //ec = boost::system::error_code();
    }
    return static_cast<int>( mysql_affected_rows(c) ); 
}

std::string db_escape(const std::string& param, const dbc::connection& wc)
{
    if (param.empty()) {
        return param;
    }
    // AUTO_CPU_TIMER("sql:escape");

    std::string escaped(2* param.size() + 1,0);

    native_handle_type c = wc.native_handle();
    unsigned long n = mysql_real_escape_string(c, const_cast<char*>(escaped.c_str()),param.c_str(),param.size());
    // if (0 == mysql_real_escape_string(c, const_cast<char*>(escaped.c_str()),param.c_str(),len))
    if (n == 0)
    {
        LOG << "MySQL query "<< mysql_errno(c) << mysql_error(c);
        // THROW_EX(EN_SQL_Fail);
        BOOST_THROW_EXCEPTION( sql::error_query() );
    }

    escaped.resize(n);
    return escaped;
}

// unsigned int datas::_index(const string& field)
// {
//     if (fields_.empty())
//     {
//         unsigned int num_fields;
//         unsigned int i;
//         MYSQL_FIELD *fields;
// 
//         num_fields = mysql_num_fields(result_);
//         fields = mysql_fetch_fields(result_);
//         fields_.reserve(num_fields);
//         clog << format("Num rows=%u fields=%u\n") % mysql_num_rows(result_) % num_fields;
//         for (i = 0; i < num_fields; i++)
//         {
//             fields_.push_back(fields[i].name);
//             // clog << format("Field %u is %s\n") % i % fields[i].name;
//         }
//         // if ( (ec = mysql_errno(my)) != 0)
//         // {
//         //     result_ = -1;
//         // }
//     }
//     for (unsigned int i = 0; i < fields_.size(); ++i)
//     {
//         // clog << format("fieldx %s\n") % fields_[i];
//         if (fields_[i] == field)
//             return i;
//     }
//     return fields_.size();
// }

//initializer::initializer (const boost::property_tree::ptree & ini)
//{
//    sql::dbc::init( ini ); //getcfg(cfg, "database")
//}
//initializer::~initializer()
//{
//    LOG << __FILE__ << __LINE__;
//}

} // namespace 

// std::ostream& operator<<(std::ostream& outs, const std::vector<std::string>& c)
// {
//     std::vector<std::string>::const_iterator it = c.begin();
//     for ( ; it != c.end(); ++it)
//         outs << *it << " ";
//     return outs;
// }

//typedef boost::shared_ptr<redisReply> redis_reply_ptr;
struct hiredis;

// #include <asio/ip/tcp.hpp> namespace ip = boost::asio::ip;

static std::string redis_host_ = DEFA_REDIS_HOST;
static unsigned short redis_port_ = DEFA_REDIS_PORT;

namespace redis {

void init(const property_tree::ptree& ini)
{
    redis_host_ = ini.get_optional<std::string>("host").get_value_or(DEFA_REDIS_HOST);
    redis_port_ = ini.get_optional<unsigned short>("port").get_value_or(DEFA_REDIS_PORT);
    LOG << "redis" << redis_host_ << redis_port_;
}

// typedef boost::container::flat_multimap<endpoint, context_ptr> multimap_t;
typedef std::multimap<endpoint, context_ptr> multimap_t;

struct context_mgr : multimap_t
{
    static context_ptr make(endpoint const& ep);
    static void save(endpoint const& ep, context_ptr ptr);

private:
    static void free(redisContext *redx);
    static context_ptr real_make(endpoint const& ep);

    boost::mutex mutex_;

    ~context_mgr();
    context_mgr() {}
    static context_mgr& instance();
};

context_mgr::~context_mgr()
{
    // redisFree(redx);
}

void context_mgr::free(redisContext *redx)
{
    LOG << redx ; // << gettid();
    redisFree(redx);
}

context_mgr& context_mgr::instance()
{
    static context_mgr m;
    return m;
}

context_ptr context_mgr::real_make(endpoint const& ep)
{
    AUTO_CPU_TIMER("redis:connect");
    redisContext *redx = 0;

    LOG << "redis connect" << ep;
    while (!redx)
    {
        struct timeval tv = { 1, 1000*0 };
        redx = redisConnectWithTimeout(ep.first.c_str(), ep.second, tv);
        if (redx && redx->err)
        {
            LOG << "redis connect error:" << redx->errstr;
            redisFree(redx);
            redx = 0;
            sleep(1);
        }
    }

    return context_ptr(redx, &context_mgr::free); // return context_helper(cp, ep);
}

context_ptr context_mgr::make(endpoint const& ep)
{
    context_mgr& my = instance();
    {
        boost::unique_lock<boost::mutex> scoped_lock(my.mutex_);

        auto it = my.find(ep);
        if (it != my.end())
        {
            context_ptr ptr = it->second;
            my.erase(it);
            return ptr; //context_helper(ptr, ep);
        }
    }

    return real_make(ep); //context_helper(real_make(ep), ep);
}

void context_mgr::save(endpoint const& ep, context_ptr ptr)
{
    context_mgr& my = instance();
    boost::unique_lock<boost::mutex> scoped_lock(my.mutex_);

    my.insert(std::make_pair(ep, ptr));
}

context::context()
    : context_helper(endpoint(redis_host_,redis_port_))
{
}

context::context(std::string const& h, unsigned short p)
    : context_helper(endpoint(h,p))
{
}

context::~context()
{
}

context_helper::context_helper(endpoint const& ep)
    : context_ptr_(context_mgr::make(ep))
    , endp_(ep)
    , cput_("redis:context")
{
    n_reply_ = 0;
}

redis::reply context_helper::reply()
{
    // AUTO_CPU_TIMER("redis:reply");
    void *rp;
    int ret;

    ret = redisGetReply(context_ptr_.get(), &rp);
    if (ret != REDIS_OK)
    {
        LOG << "redis error" << endp_;
        n_reply_ = 0;
        context_ptr_.reset();
        return redis::reply();
    }

    --n_reply_;
    redis::reply rpy(static_cast<redisReply*>(rp), freeReplyObject);
    switch (rpy->type)
    {
        case REDIS_REPLY_INTEGER:
            LOG << "int" << rpy->integer;
            break;
        case REDIS_REPLY_STRING:
            LOG << "str" << rpy->str;
            break;
        case REDIS_REPLY_ARRAY:
            LOG << "array" << rpy->elements;
            break;
        case REDIS_REPLY_NIL:
            LOG << "NIL";
            break;
        default:
            LOG << "unknown" << rpy->type;
            break;
    }
    return rpy;
}

context_helper::~context_helper()
{
    // LOG << n_reply_ << context_ptr_.use_count();
    if (context_ptr_) //( && context_ptr_.use_count() == 1)
    {
        if (n_reply_ > 0)
        {
            LOG << "redis ignore" << n_reply_;
            while (n_reply_ > 0) {
                redis::reply rpy = this->reply();
                if (!rpy) {
                    context_ptr_.reset();
                    break;
                }
            }
        }
        BOOST_ASSERT (n_reply_ == 0);
        LOG << "redis" << endp_ << n_reply_ << bool(context_ptr_);

        if (context_ptr_) {
            context_mgr::save(endp_, context_ptr_); // contexts().insert(std::make_pair(endp_, context_ptr_));
        }
    }
}

//context_helper context_helper::make()
//{
//    return context_helper::make(redis_host_, redis_port_);
//}
//
//context_helper context_helper::make(std::string const & host, unsigned short port)
//{
//    return context_mgr::make( std::make_pair(host,port) );
//}

} // namespace redis


#if 0 // mongodb
namespace MyMongo
{
    std::string mongo_conn_mgr::mongo_host_ = DEFA_MONGO_HOST;
    unsigned short mongo_conn_mgr::mongo_port_ = DEFA_MONGO_PORT;

    void mongo_conn_mgr::init(const property_tree::ptree& ini)
    {
        mongo::Status status = mongo::client::initialize();
        if ( !status.isOK() ) {
            LOG<< "failed to initialize the client driver: " << status.toString();
            BOOST_THROW_EXCEPTION( error_init() );
        }

        mongo_host_ = ini.get_optional<std::string>("host").get_value_or(mongo_host_);
        mongo_port_ = ini.get_optional<unsigned short>("port").get_value_or(mongo_port_);
        LOG << "mongo" << mongo_port_ << mongo_port_;
    }

    mongo_conn_mgr& mongo_conn_mgr::inst()
    {
        static mongo_conn_mgr ins;
        return ins;
    }

    bool mongo_conn_mgr::create_connect( int type )
    {
        Mongo_Conn_Ptr c = boost::make_shared<mongo::DBClientConnection>();
        c->connect(mongo_host_ + ":" + boost::lexical_cast<std::string>(mongo_port_));

        m_conn_pool_.push_back( make_pair( c, false ) );

        return true;
    }

    mongo_conn_mgr::Mongo_Conn_Ptr mongo_conn_mgr::get_connect( int type )
    {
        Mongo_Conn_Ptr ptr;
        BOOST_FOREACH ( auto &i, m_conn_pool_) {
            if ( !i.second ) {
                i.second = true;
                ptr = i.first;
                break;
            } 
        }

        if ( !ptr && create_connect( type ) ) {
            auto& i = *m_conn_pool_.rbegin();
            i.second = true;
            ptr = i.first;
        }

        return ptr;
    }

    void mongo_insert( const std::string& db_table, const mongo::BSONObj& in )
    {
        auto c = mongo_conn_mgr::inst().get_connect();
        c->insert(db_table, in);
    }

    boost::shared_ptr<mongo::DBClientCursor> mongo_select(const std::string& db_table, const mongo::Query& in )
    {
        auto c = mongo_conn_mgr::inst().get_connect();
        boost::shared_ptr<mongo::DBClientCursor> cursor( c->query(db_table, mongo::BSONObj()));
        if (!cursor.get()) {
            LOG << "query failure";
            BOOST_THROW_EXCEPTION( error_query() );
        }

        return cursor;
    }
}
#endif // mongodb

