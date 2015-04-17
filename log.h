#ifndef _LOG_H__
#define _LOG_H__

#include <syslog.h>
#include <cstdint>
#include <set>
#include <vector>
#include <ostream>
#include <utility>
#include <boost/array.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/ref.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/noncopyable.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/timer/timer.hpp>
#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

inline std::ostream& operator<<(std::ostream& out, boost::asio::const_buffer const & buf)
{
    out.write(boost::asio::buffer_cast<char const*>(buf), boost::asio::buffer_size(buf));
    return out;
}
inline std::ostream& operator<<(std::ostream& out, boost::asio::const_buffers_1 const & buf)
{
    boost::asio::const_buffer const & cb = buf;
    return out << cb;
}

template <typename T, size_t N>
std::ostream& operator<<(std::ostream& out, boost::array<T,N> const& v)
{
    if (!v.empty())
    {
        auto it = v.begin();
        out << *it;
        for (++it; it != v.end(); ++it)
            out << " " << *it;
    }
    return out;
}
template <typename T>
std::ostream& operator<<(std::ostream& out, std::vector<T> const& v)
{
    if (!v.empty())
    {
        auto it = v.begin();
        out << *it;
        for (++it; it != v.end(); ++it)
            out << " " << *it;
    }
    return out;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, std::set<T> const& v)
{
    if (!v.empty())
    {
        auto it = v.begin();
        out << *it;
        for (++it; it != v.end(); ++it)
            out << " " << *it;
    }
    return out;
}

template <typename X, typename Y>
std::ostream & operator<<(std::ostream & out, std::pair<X,Y> const & p)
{
    return out << p.first <<" "<< p.second;
}

namespace logging {

extern bool syslog_;

template<typename Container>
struct logger2_sink
{
    typedef typename Container::value_type  char_type;
    typedef boost::iostreams::sink_tag      category;

    logger2_sink(Container& container) : container_(container) { }

    std::streamsize write(const char_type* s, std::streamsize n)
    {  
        std::streamsize m = container_.capacity() - container_.size();
        if (m > 0) {
            container_.insert(container_.end(), s, s + std::min(m, n));
        }
        // std::cout << &container_ <<" "<< container_.capacity() <<" "<< container_.size() << "\n";
        return n;
    }
    // Container& container() { return container_; }
private:
    logger2_sink operator=(const logger2_sink&);
    Container& container_;
};

template<class Conf>
struct logger_stream : boost::noncopyable
    , boost::iostreams::stream<logger2_sink<boost::container::static_vector<char,Conf::capacity> > >
    , boost::container::static_vector<char,Conf::capacity>
{
    typedef boost::container::static_vector<char,Conf::capacity> bufs_t;
    typedef boost::iostreams::stream<logger2_sink<boost::container::static_vector<char,Conf::capacity> > > stream_base;

    logger_stream() : stream_base(*this) // ( static_cast<bufs_t&>(*this) )
        { sizepfx=0; }

    void commit(int line, char const *name);

    template <typename X>
    void prefix(X const& x)
    {
        bufs_t& bufs = *this;
        if (sizepfx >= 1)
        {
            bufs.resize(bufs.size() - 1);
            *this << " ";
        }
        *this << x << ":";
        sizepfx = bufs.size();
    }

    template <typename X, typename ...Args>
    void prefix(X const& x, Args const& ...params)
    {
        prefix(x);
        prefix(params...);
    }

    // boost::container::static_vector<char,Conf::capacity> bufs;
    // logger2_sink sink;
    unsigned short sizepfx, _;
    boost::mutex mutex_;

    static logger_stream<Conf> instance;
};
template <typename Conf> logger_stream<Conf> logger_stream<Conf>::instance;

template<class Conf>
void logger_stream<Conf>::commit(int line, char const *name)
{
    this->flush();
    bufs_t& bufs = *this;
    // std::cout << &bufs <<" "<< bufs.capacity() <<" "<< bufs.size() <<" "<< sizepfx <<"\n";
    if (bufs.size() > sizepfx)
    {
        if (syslog_)
        {
            int len = bufs.size();
            char *p = &bufs[0];
            char const* fmt = "%.*s";
            if (line && name)
                fmt = "%.*s #%d:%s";
            ::syslog(LOG_USER|LOG_INFO, fmt, len, p, line, name);
        }
        else
        {
            std::clog.write(&bufs[0], bufs.size());
            if (line && name)
                std::clog << " #"<<line <<":"<< name;
            std::clog << "\n";
        }
        bufs.resize(sizepfx); // clear(); seekp(sizepfx, std::ios::beg);
    }
}

template <typename Conf>
struct lock_helper : boost::unique_lock<boost::mutex>
{
    lock_helper(logger_stream<Conf>& ls)
        : boost::unique_lock<boost::mutex>(ls.mutex_)
        , ls_(&ls)
    {}

    std::ostream& stream() const { return *ls_; }
    void commit(int ln, char const* nm) const { ls_->commit(ln, nm); }

    logging::logger_stream<Conf>* ls_;
};

template <typename Conf>
struct logger_helper : lock_helper<Conf>
{
    int line_;
    const char* name_;

    template <typename X>
    logger_helper<Conf> const& operator<<(X const& x) const
    {
        static_cast<std::ostream&>(this->stream()) <<" "<< x;
        return *this;
    }

    ~logger_helper()
    {
        this->commit(line_, name_);
    }

    logger_helper(int line, const char* nm)
        : lock_helper<Conf>(logger_stream<Conf>::instance)
    {
        line_ = line;
        name_ = nm;
    }
};

//template <typename Ls>
//inline logger_helper<Ls> make_logger_helper(Ls* ls, int ln, char const* nm)
//{
//    return logger_helper<Ls>(ls,ln,nm);
//}

template <typename Int> void syslog(Int opt, Int facility)
{
    logging::syslog_ = 1;
    ::openlog(0, opt, facility);
}

struct info
{
    BOOST_STATIC_CONSTANT(int, level=LOG_INFO);
    BOOST_STATIC_CONSTANT(int, capacity=1024);
};

struct warning
{
    BOOST_STATIC_CONSTANT(int, level=LOG_WARNING);
    BOOST_STATIC_CONSTANT(int, capacity=1024);
};

} // namespace logging

#define LOG  if(1)logging::logger_helper<logging::info>(__LINE__,__FUNCTION__)
#define LOG_I LOG
#define LOG_W if(1)logging::logger_helper<logging::warning>(__LINE__,__FUNCTION__)

struct auto_cpu_timer_helper : boost::timer::auto_cpu_timer
{
    auto_cpu_timer_helper(std::string const& tag);
};

#define CONCAT_LINENO_(cls,n) cls(cls ## __ ## n ## _)
#define CLS_DECL_(cls,n) CONCAT_LINENO_(cls,n)
#define AUTO_CPU_TIMER(x) CLS_DECL_(auto_cpu_timer_helper,__LINE__)(x)

struct daily_filename_weekday
{
    daily_filename_weekday(boost::filesystem::path const& d)
        : dir_(d)
    {
        if (!boost::filesystem::exists(dir_)) {
            boost::filesystem::create_directories(dir_);
        }
    }
    boost::filesystem::path operator()(struct tm const& tm) const
    {
        return (dir_ / boost::lexical_cast<std::string>(tm.tm_wday));
    }
    boost::filesystem::path dir_;
};

struct daily_filename_monthday
{
    daily_filename_monthday(boost::filesystem::path const& d)
        : dir_(d)
    {
        if (!boost::filesystem::exists(dir_)) {
            boost::filesystem::create_directories(dir_);
        }
    }
    boost::filesystem::path operator()(struct tm const& tm) const
    {
        char s[4];
        sprintf(s, "%02d", tm.tm_mday);
        return (dir_ / s); //(dir_ / boost::lexical_cast<std::string>(tm.tm_mday));
    }
    boost::filesystem::path dir_;
};

template <typename Fname, int H=0>
struct daily_rotate_ofstream : boost::noncopyable
{
    template <typename Fend, typename Fbegin>
    boost::filesystem::ofstream& make(Fend* day_end, Fbegin* day_begin);
    boost::filesystem::ofstream& make()
    {
        void (*fp)(std::ostream&) = 0;
        return make(fp,fp);
    }
    boost::filesystem::ofstream& ostream() { return ofs_; }

    template <typename ...V>
    daily_rotate_ofstream(V... a)
        : fname_(a...)
    {
        tp_day_begin_=0;
    }

    Fname fname_;
    boost::filesystem::ofstream ofs_;
    time_t tp_day_begin_;

    static time_t mktime_align(struct tm const& p)
    {
        struct tm tm = p;
        tm.tm_hour = tm.tm_min = tm.tm_sec = 0;
        tm.tm_wday = 0;
        tm.tm_yday = 0;
        return mktime(&tm);
    }
};

template <typename Fname,int H>
template <typename Fend, typename Fbegin>
boost::filesystem::ofstream& daily_rotate_ofstream<Fname,H>::make(Fend* day_end, Fbegin* day_begin)
{
    enum { SECONDS_PDAY=(60*60*24) };
    time_t tpc = time(0);

    std::ios::openmode mode = std::ios::app;
    if (tp_day_begin_)
    {
        if (tpc - tp_day_begin_ < SECONDS_PDAY) {
            return ofs_;
        }
        if (day_end) {
            (*day_end)(ofs_);
        }
        tp_day_begin_ += SECONDS_PDAY;

        ofs_.flush();
        ofs_.close();
        ofs_.clear();
        mode = std::ios::out;

    } else {
        struct tm tm = {};
        time_t tpa = tpc - (60*60*H);
        localtime_r(&tpa, &tm);
        tp_day_begin_ = mktime_align(tm) + (60*60*H);
        BOOST_ASSERT(tp_day_begin_ <= tpc);
    }

    {
        struct tm tm = {};
        localtime_r(&tp_day_begin_, &tm);
        auto fn = fname_(tm);
        if (mode == std::ios::app) {
            if (boost::filesystem::exists(fn)) {
                day_begin = 0;
            }
        }

        ofs_.open(fn, mode);
        if (day_begin) {
            (*day_begin)(ofs_);
        }
    }
    return ofs_;
}

typedef int64_t Int64;

Int64 milliseconds();

#endif

