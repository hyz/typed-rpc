#include "config.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <iostream>
#include <boost/asio/streambuf.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>

#include "log.h"

using namespace std;
using namespace boost;

//std::ostream& operator<<(std::ostream& out, boost::asio::const_buffers_1 const & bufs)
//{
//    BOOST_FOREACH(boost::asio::const_buffer const& buf, bufs)
//        out << buf;
//    return out;
//}

namespace logging {

bool syslog_ = 0;

extern std::ostream * ostream_ptr_;

struct ostream_fwd
{
    explicit ostream_fwd(int pri, const char* fn, int ln);

    ~ostream_fwd();

    const ostream_fwd& operator<<(std::ostream& (*pf)(std::ostream&)) const
    {
        *ostream_ptr_ << pf;
        return *this;
    }

    template <typename T> const ostream_fwd& operator<<(const T& x) const
    {
        *ostream_ptr_ << x;
        return *this;
    }

    unsigned short pri_;
    unsigned short ln_;
    const char* fn_;
};

} // namespace
namespace logging {

std::ostream *ostream_ptr_ = 0;
static boost::asio::streambuf *logsbuf_ = 0;

//boost::reference_wrapper<std::ostream> clog_(std::clog);
// static boost::mutex mutex_;
// static bool syslog_ = false;
// static std::ostream* clog_ = &std::clog;

ostream_fwd::ostream_fwd(int pri, const char* fn, int ln)
{
    pri_ = pri;
    ln_ = ln;
    fn_ = fn;
    // mutex_.lock();
}

ostream_fwd::~ostream_fwd()
{
    // std::clog << static_cast<void*>(std::clog.rdbuf()) <<":"<< static_cast<void*>(&logsbuf_) << std::endl;
    if (logsbuf_)
    {
        asio::streambuf::const_buffers_type bufs = logsbuf_->data();
        asio::streambuf::const_buffers_type::const_iterator i = bufs.begin();
        if (i != bufs.end())
        {
            size_t n = 0;
            for (; i != bufs.end(); ++i)
            {
                auto& b = *i;
                void const *p = boost::asio::buffer_cast<void const*>(b); // asio::detail::buffer_cast_helper(*i);
                size_t len = boost::asio::buffer_size(b); // asio::detail::buffer_size_helper(*i);

                ::syslog(pri_, "%.*s #%s:%d", (int)len, (char*)p, fn_, ln_);
                n += len;
            }
            logsbuf_->consume(n);
        }
    }
    else
    {
        struct tm tm;

        time_t ss = time(0);
        localtime_r(&ss, &tm);

        char ts[32] = {0};
        strftime(ts,sizeof(ts)-1, "%T", &tm);

        (*ostream_ptr_) <<" #"<< fn_ <<":"<<ln_<<" "<< ts <<"\n";
    }

    // mutex_.unlock();
}

void setup(std::ostream * outs, int opt, int facility)
{
    if (outs)
    {
        logsbuf_ = 0;
        ostream_ptr_ = outs;
    }
    else
    {
        static boost::asio::streambuf logsbuf_s;
        static std::ostream slog_s(&logsbuf_s);
        openlog(0, opt, facility);
        logsbuf_ = &logsbuf_s;
        ostream_ptr_ = &slog_s;
    }

    // static asio::streambuf outbuf; cout.rdbuf(&outbuf);
    //boost::reference_wrapper<std::ostream> 
    //clog_ = boost::ref(slog_);

    // syslog_ = true;
    // ostream_fwd::clog_ = &slog;
}

} // namespace

extern std::ostream & auto_cpu_timer_ostream();

#ifndef AUTO_CPU_TIMER_OSTREAM
# ifndef RUN_DIR
# define RUN_DIR "/tmp"
# endif
std::ostream & auto_cpu_timer_ostream()
{
    static daily_rotate_ofstream<daily_filename_weekday> ofs(RUN_DIR "/cpu-time");
    return ofs.make();
}
#endif

using namespace boost::posix_time;

auto_cpu_timer_helper::auto_cpu_timer_helper(std::string const& tag)
    : boost::timer::auto_cpu_timer(
            auto_cpu_timer_ostream(),
            str(boost::format("%1%\t%2%\t%%w\n") % second_clock::local_time() % tag))
{
    // second_clock::local_time()
}

