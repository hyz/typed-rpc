#ifndef UTIL_H__
#define UTIL_H__

#include <time.h>
#include <string>
#include <stdexcept>
#include <functional>
#include <boost/assert.hpp>
#include <boost/static_assert.hpp>
#include <boost/range.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/find.hpp>

#include <iconv.h>

int sendsms(const std::string& phone, const std::string& message);

std::string path_leaf(const std::string& path);

std::string urldecode(const std::string &str_source);
std::string urldecode(const char* beg, const char* end);
std::string urlencode(const std::string &str_source);

template <typename in_type, typename out_type>
out_type& wiconv(out_type& out, const char* out_code, in_type& in, const char* in_code)
{
    BOOST_STATIC_ASSERT(sizeof(wchar_t) == 4);

    char* ibuf = reinterpret_cast<char*>(const_cast<typename in_type::value_type*>(&in[0]));
    size_t ileft = in.size() * sizeof(typename in_type::value_type);

    char* obuf = reinterpret_cast<char*>(&out[0]);
    size_t oleft = out.size() * sizeof(typename out_type::value_type);

    iconv_t cd = iconv_open(out_code, in_code);
    if (cd == (iconv_t)-1)
    {
        throw std::runtime_error("iconv_open error");
    }
    size_t res = iconv(cd, &ibuf, &ileft, &obuf, &oleft);
    iconv_close(cd);
    if (res == (size_t)-1)
    {
        throw std::runtime_error("iconv error");
    }

    BOOST_ASSERT(oleft % sizeof(typename out_type::value_type) == 0);
    out.resize(out.size() - oleft/sizeof(typename out_type::value_type));

    return out;
}

inline std::wstring utf8to32(std::string const& s)
{
    std::wstring w(s.size(), '\0');
    return wiconv(w, "utf-32", s, "utf-8");
}

inline std::string utf32to8(std::wstring const& s)
{
    std::string n(s.size() * 4, '\0');
    return wiconv(n, "utf-8", s, "utf-32");
}

unsigned int randuint(unsigned int beg=0, unsigned int end = std::numeric_limits<unsigned int>::max());

std::string readfile(boost::filesystem::path const & fp);
void copyfile(std::string const & src, std::string const & dst);

template <typename Self, typename Iter>
Self& for_self(Self& sf, Iter begin, Iter end, Iter * iend=0)
{
    for (; begin != end; ++begin)
        if (!sf(*begin))
            break;
    if (iend)
        *iend = begin;
    return sf;
}

template <int N> std::string time_format(char const * sfmt, struct tm& tm)
{
    char tmp[N];
    strftime(tmp,sizeof(tmp), sfmt, &tm);
    return std::string(tmp);
}
template <int N> std::string time_format(std::string const & sfmt, struct tm& tm)
{
    return time_format<N>(sfmt.c_str(), tm);
}

template <int N> std::string time_format(char const * sfmt, time_t tp=0)
{
    if (tp == 0)
        tp = time(0);
    struct tm tm;
    localtime_r(&tp, &tm);
    return time_format<N>(sfmt, tm);
}
template <int N> std::string time_format(std::string const & sfmt, time_t tp=0)
{
    return time_format<N>(sfmt.c_str(), tp);
}

inline std::string time_string(time_t tp=0)
{
    return time_format<64>("%F %T", tp);
}

template <typename T> inline T& zeros(T& a)
{
    memset(&a, 0, sizeof(T));
    return a;
}

template <typename Iter>
std::string base64_decode(Iter begs, Iter ends)
{
    using namespace boost::archive::iterators;

    typedef transform_width<binary_from_base64<Iter>,8,6> to_binary_iter;

    int npad = 0;
    for (auto it = ends; it != begs && (*(it-1) == '='); --it) {
        ++npad;
    }
    // unsigned int npad = std::count(begs, ends, '=');

    std::string result;
    result.assign(to_binary_iter(begs), to_binary_iter(ends));
    result.erase(result.end() - npad, result.end());

    return result; //std::string(to_binary_iter(beg), to_binary_iter(end)); // decode
}

inline std::string base64_decode(std::string const & s)
{
    return base64_decode(s.begin(), s.end());
}

template <typename Iter>
std::string base64_encode(Iter begs, Iter ends)
{
    using namespace boost::archive::iterators;
    typedef base64_from_binary<transform_width<Iter,6,8>> to_base64_iter;
    return std::string(to_base64_iter(begs), to_base64_iter(ends));
}

inline std::string base64_encode(std::string const & s)
{
    return base64_encode(s.begin(), s.end());
}

template <typename Rng, typename Pred>
bool is_ordered(Rng const& rng, Pred const& pred)
{
    if (!boost::empty(rng))
    {
        auto prev = boost::begin(rng), end = boost::end(rng);
        auto it = prev;
        for (++it; it != end; ++prev, ++it)
            if (!pred(*prev, *it))
                return false;
    }
    return true;
}

struct scope_exit
{ 
    scope_exit(std::function<void (void)> f) : f_(f) {} 
    ~scope_exit(void) { f_(); } 
private:
    std::function<void (void)> f_;
};


template <typename Rng>
typename Rng::const_iterator find_anys(std::string const& txt, Rng const& subs)
{
    auto it = subs.begin();
    auto end = subs.end();
    while (it != end) {
        auto r = boost::algorithm::find_first(txt, *it);
        if (!boost::empty(r)) {
            return it;
        }
        ++it;
    }
    return end;
}

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

    template <int X> void b8(bool y) {
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


#endif

