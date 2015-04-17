#ifndef __JOSN_H__
#define __JOSN_H__

#include <stdio.h>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
//#include <list>
#include <stack>
#include <type_traits>
//#include <stdexcept>
// #include <map>
#include <boost/foreach.hpp>
#include <boost/container/flat_map.hpp>
// #include <boost/assign/list_inserter.hpp>
#include <boost/range.hpp>
#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/none.hpp>
//#include <boost/type_traits/is_enum.hpp>
//#include <boost/type_traits/is_floating_point.hpp>
//#include <boost/type_traits/is_integer.hpp>
//#include <boost/type_traits/is_integral.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/optional/optional.hpp>

#include "myerror.h"

namespace json {
// using boost::assign::insert;

// using boost::none;
typedef boost::none_t null_t;
typedef bool bool_t;
typedef double float_t;
typedef int64_t int_t;
typedef std::string string;

struct object;
struct array;

typedef boost::variant<object,array,string,int_t,bool,double,null_t> variant;

namespace types {
    typedef json::null_t null_t;
    typedef json::bool_t bool_t;
    typedef json::int_t int_t;
    typedef json::float_t float_t;
    typedef json::string string;
    typedef json::object object;
    typedef json::array array;
    typedef json::variant variant;
} // namespace types

struct error : ::myerror {};
struct error_key : error {};
struct error_index : error {};
struct error_value : error {};
struct error_decode : error {};

typedef boost::error_info<struct json_key,std::string> errinfo_key;
typedef boost::error_info<struct json_index,size_t> errinfo_index;
typedef boost::error_info<struct json_index,std::string> errinfo_json;

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

//template<typename _Tp>
//constexpr int_t forward(typename std::remove_reference<_Tp>::type x
//        , typename std::enable_if<boost::is_integer<_Tp>::value>::type* =0) noexcept
//{ return int_t(x); }
//constexpr typename boost::enable_if<boost::mpl::or_<boost::is_integer<_Tp>,boost::is_enum<_Tp>>,int_t>::type

template <typename A,typename T> struct const_as { typedef T type; };
template <typename A,typename T> struct const_as<const A,T> { typedef const T type; };

template <bool,typename T> struct const_if { typedef T type; };
template <typename T> struct const_if<true,T> { typedef const T type; };

template <typename T> struct is_pair : std::false_type {};
template <typename K, typename V> struct is_pair<std::pair<K, V>> : std::true_type {};
template <typename T> struct is_bool : std::false_type {};
template <> struct is_bool<bool> : std::true_type {};

template <typename V, typename Tv, typename Ti>
struct jsforward {
    constexpr static Ti from(V&& v) { return Ti(v); }
};
template <typename V, typename Ti>
struct jsforward<V,Ti,Ti> {
    constexpr static V&& from(V&& v) { return std::forward<V>(v); }
};

template <typename T>
struct inner_type {
    typedef typename std::conditional<is_bool<T>::value
        , bool
        , typename std::conditional<std::is_enum<T>::value
            , json::int_t
            , typename std::conditional<std::is_integral<T>::value
                , json::int_t
                , typename std::conditional<std::is_floating_point<T>::value
                    , double
                    , typename std::conditional<std::is_same<char*,T>::value
                        , std::string
                        , T
                    >::type
                >::type
            >::type
        >::type
    >::type type;
};

//template <typename V>
//constexpr typename std::enable_if<
//        std::is_same<typename std::decay<V>::type,typename inner_type<typename std::decay<V>::type>::type>::value
//    , V>::type&&
//fwd2(V&& v) { return std::forward<V>(v); }
//template <typename V>
//constexpr typename std::enable_if<
//        !std::is_same<typename std::decay<V>::type,typename inner_type<typename std::decay<V>::type>::type>::value
//    , typename inner_type<typename std::decay<V>::type>::type
//fwd2(V&& v) { return typename inner_type<typename std::decay<V>::type>::type(v); }

typedef std::vector<json::variant> array_impl_t;
struct array : array_impl_t
{
    typedef size_t index_type;

    template <typename T> iterator insert(iterator p, T&& v);

    template <typename T> iterator push_back(T&& x)
    {
        return this->insert(end(), std::forward<T>(x));
    }

    iterator iterator_to(size_t k);
    const_iterator iterator_to(size_t k) const { return const_cast<array*>(this)->iterator_to(k); }
};

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

typedef boost::container::flat_map<std::string, json::variant> object_impl_t;
struct object : object_impl_t
{
    typedef std::string index_type;

    bool rename(const std::string & fr, const std::string & to);

    template <typename K, typename V>
    std::pair<iterator,bool> emplace(K&& k, V&& v)
    {
        //typedef typename std::remove_cv<typename std::remove_reference<V>::type>::type Tv;
        typedef typename std::decay<V>::type Tv;
        typedef typename inner_type<Tv>::type Ti;

        return object_impl_t::emplace(std::forward<K>(k)
                , variant( jsforward<V,Tv,Ti>::from(std::forward<V>(v)) ));
    }

    iterator iterator_to(std::string const& k) { return find(k); }
    const_iterator iterator_to(std::string const& k) const { return find(k); }
};

inline array::iterator array::iterator_to(size_t k)
{
    return (k < size() ? begin()+k : end());
}

template <typename T>
inline array::iterator array::insert(iterator it, T&& v)
{
    typedef typename std::decay<T>::type Tv;
    typedef typename inner_type<Tv>::type Ti;
    return array_impl_t::insert(it, variant( jsforward<T,Tv,Ti>::from(std::forward<T>(v)) ));
}

template <typename T>
void assign(variant& var, T&& v)
{
    typedef typename std::decay<T>::type Tv;
    typedef typename inner_type<Tv>::type Ti;
    var = variant( jsforward<T,Tv,Ti>::from(std::forward<T>(v)) );
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

struct object_inserter
{
    template <typename K, typename V>
    object_inserter operator()(K&& k, V&& v) const
    {
        jo_->emplace(std::forward<K>(k), std::forward<V>(v));
        return *this;
    }
    object_inserter(object & jo) : jo_(&jo) {}
    object * jo_;
};

struct object_replacer
{
    template <typename K, typename V> object_replacer operator()(K&& k, V&& v) const
    {
        auto p = jo_->emplace(std::forward<K>(k),std::forward<V>(v));
        if (!p.second) {
            assign(p.first->second, std::forward<V>(v));
        }
        return *this;
    }
    object_replacer(object & jo) : jo_(&jo) {}
    object * jo_;
};

inline object_inserter insert (object & jo) { return object_inserter(jo); }
inline object_replacer replace(object & jo) { return object_replacer(jo); }

//template <class T> class traits<T, typename enable_if<is_float<T> >::type> { };

//template< typename T >
//void assign(variant& var, T&& val
//    /*, typename boost::enable_if< boost::mpl::or_< boost::is_integer<T> , boost::is_enum<T> > >::type**/)
//{
//    typedef typename traits<T>::value_type value_type;
//    var = value_type(std::forward<T>(val));
//}

//template <typename T>
//typename traits<T>::optional_type const as(variant const& var
//    /*, typename boost::disable_if< boost::mpl::or_< boost::is_integer<T> , boost::is_enum<T> > >::type* = 0*/)
//{
//    typedef typename traits<T>::value_type value_type;
//    typedef typename traits<T>::optional_type optional_type;
//    if (auto ov = boost::get<value_type>(&var))
//        return optional_type(*ov);
//    return optional_type();
//}

//template <typename T>
//typename traits<T>::optional_type const as(variant const& var
//    , typename boost::enable_if< boost::mpl::or_< boost::is_integer<T> , boost::is_enum<T> > >::type* = 0)
//{
//    if (auto ov = boost::get<int_t>(&var))
//        return typename traits<T>::optional_type(*ov);
//    return typename traits<T>::optional_type();
//}

//template <typename T, typename O, typename K>
//typename traits<typename std::add_const<T>::type>::optional_type as(O const& o, K const& k)
//{
//    auto i = o.iterator_to(k);
//    if (i == o.end())
//        return typename traits<T>::optional_type();
//    return as<T>(value_ref(i));//(i->second);
//}

//template <typename T>
//typename traits<T>::optional_type as(array const& ja, size_t x)
//{
//    typedef typename traits<T>::value_type value_type;
//    typedef typename traits<T>::optional_type optional_type;
//    if (x >= ja.size())
//        BOOST_THROW_EXCEPTION( json::error_index() << errinfo_index(x) );
//    return as<T>(ja[x]);
//}

// template< typename T >
// boost::optional<T const&> as(array const& ja, size_t x
//     , typename boost::disable_if< boost::mpl::or_< boost::is_integer<T> , boost::is_enum<T> > >::type* = 0)
// {
//     return get<T>(ja, x);
// }
// 
// template< typename T >
// boost::optional<T> as(array const& ja, size_t x
//     , typename boost::enable_if< boost::mpl::or_< boost::is_integer<T> , boost::is_enum<T> > >::type* = 0)
// {
//     if (auto const* ov = get<json::int_t>(&ja, x))
//         return boost::optional<T>(*ov);
//     return boost::optional<T>();
// }

template <typename T>
typename inner_type<T>::type& ref(variant& o)
{
    typedef typename inner_type<T>::type Ti;
    Ti* p = boost::get<Ti>(&o);
    if (!p) {
        BOOST_THROW_EXCEPTION( json::error_value() );
    }
    return *p;
}
template <typename T>
typename inner_type<T>::type const& ref(variant const& o)
{
    return ref<T>(const_cast<variant&>(o));
}

//template <typename T, typename I>
//auto ref(I it)//(, typename std::iterator_traits<I>::pointer =0)
//    -> typename std::enable_if< is_pair<typename std::iterator_traits<I>::value_type>::value
//        , decltype(ref<T>(it->second))>::type
//    //-> typename const_if<std::is_const<typename std::remove_reference<typename std::iterator_traits<I>::reference>::type>::value
//    //        , typename inner_type<T>::type>::type&
//{
//    return ref<T>(it->second);
//}
//template <typename T, typename I>
//auto ref(I it) //(, typename std::iterator_traits<I>::pointer =0)
//    -> typename std::enable_if<!is_pair<typename std::iterator_traits<I>::value_type>::value
//        , decltype(ref<T>(*it))>::type
//    // -> decltype(ref<T>(*it))
//    //-> typename const_if<std::is_const<typename std::remove_reference<typename std::iterator_traits<I>::reference>::type>::value
//    //        , typename inner_type<T>::type>::type&
//{
//    return ref<T>(*it);
//}
template <typename T, typename P>
typename inner_type<T>::type& ref(P& p, typename std::enable_if<is_pair<P>::value>::type* =0)
{
    return ref<T>(p.second);
}
template <typename T, typename P>
typename inner_type<T>::type const& ref(P const& p, typename std::enable_if<is_pair<P>::value>::type* =0)
{
    return ref<T>(p.second);
}

template <typename T, typename O, typename K>
typename inner_type<T>::type& ref(O& o, K const& k)
{
    auto it = o.iterator_to(k);
    if (it == o.end())
        BOOST_THROW_EXCEPTION( json::error_key() << errinfo_key(k) );
    return ref<T>(*it);
}
template <typename T, typename O, typename K>
typename inner_type<T>::type const& ref(O const& o, K const& k) {
    return ref<T>(const_cast<O&>(o), k);
}

extern bool _decode(variant& result, char const* beg, char const* end);

template <typename T,typename Iter>
boost::optional<T> decode(Iter beg, Iter end)
{
    BOOST_ASSERT(beg < end);
    ((void)(end-1));

    variant var;
    if (! _decode(var, &*beg, &*end))
    {
        BOOST_THROW_EXCEPTION( json::error_decode() << errinfo_json(std::string(beg,end)) );
    }

    return boost::optional<T>( boost::move(boost::get<T>(var)) );
}

template <typename T,typename Rng>
boost::optional<T> decode(Rng const& rng)
{
    return decode<T>(boost::begin(rng), boost::end(rng));
}

template <typename T>
boost::optional<T> decode(boost::string_ref s) //(Ch const* c_str, char=Ch())
{
    return decode<T>(s.begin(), s.end()); //(c_str, c_str + std::strlen(c_str));
}
//template <typename T, typename Ch>
//boost::optional<T> decode(Ch* c_str, char=Ch())
//{
//    return decode<T>(static_cast<Ch const*>(c_str));
//}

std::string encode(json::array const & jv);
std::string encode(json::object const & jv);
std::string encode(json::variant const & jv);

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
///
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

template <typename Access> typename Access::result_type jsv_walk(json::object const& obj, Access & aces);
template <typename Access> typename Access::result_type jsv_walk(json::array const& arr, Access & aces);

template <typename Access, typename Object>
struct jsv_visitor : boost::static_visitor<typename Access::result_type>
{
    typedef typename Access::result_type result_type;
    Access* access_;
    Object const& obj_;
    typename Object::index_type const& idx_;

    jsv_visitor(Access* p, Object const& o, typename Object::index_type const& k)
        : obj_(o), idx_(k)
    { access_ = p; }

    result_type operator()(json::int_t x) { return access_->access(obj_, idx_, x); }
    result_type operator()(json::null_t x) { return access_->access(obj_, idx_, x); }
    result_type operator()(json::float_t x) { return access_->access(obj_, idx_, x); }
    result_type operator()(json::bool_t x) { return access_->access(obj_, idx_, x); }
    result_type operator()(json::string const& x) { return access_->access(obj_, idx_, x); }

    result_type operator()(json::object const& obj);
    result_type operator()(json::array const& arr);
    // bool access(json::variant const& v) { return boost::apply_visitor(*this, v); }
};

struct tag_begin {};
struct tag_end {};

template <typename Access, typename Object>
auto jsv_visitor<Access, Object>::operator()(json::object const& obj) -> result_type
{
    if (auto ret = access_->access(obj_, idx_, obj, json::tag_begin())) {
        return ret;
    }
    if (auto ret = json::jsv_walk(obj, *(this->access_))) {
        return ret;
    }
    return access_->access(obj_, idx_, obj, json::tag_end());
}

template <typename Access, typename Object>
auto jsv_visitor<Access, Object>::operator()(json::array const& arr) -> result_type
{
    if (auto ret = access_->access(obj_, idx_, arr, json::tag_begin())) {
        return ret;
    }
    if (auto ret = json::jsv_walk(arr, *(this->access_))) {
        return ret;
    }
    return access_->access(obj_, idx_, arr, json::tag_end());
}

template <typename Access>
typename Access::result_type jsv_walk(json::object const& obj, Access & aces)
{
    typedef typename Access::result_type result_type;
    BOOST_FOREACH(auto & p, obj) {
        jsv_visitor<Access,json::object> jsv(&aces, obj, p.first);
        if (result_type ret = boost::apply_visitor(jsv, p.second)) {
            return ret;
        }
    }
    return result_type();
}

template <typename Access>
typename Access::result_type jsv_walk(json::array const& arr, Access & aces)
{
    typedef typename Access::result_type result_type;
    size_t ix = 0;
    BOOST_FOREACH(auto & x, arr) {
        jsv_visitor<Access,json::array> jsv(&aces, arr, ix);
        if (auto ret = boost::apply_visitor(jsv, x)) {
            return ret;
        }
        ++ix;
    }
    return result_type();
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

template <typename RetType>
struct access_skeleton_helper
{
    typedef RetType result_type;

    template <typename T, typename K, typename V>
    result_type access(T const& p, K const& k, V const& v, json::tag_begin) const
    {
        return result_type();
    }
    template <typename T, typename K, typename V>
    result_type access(T const& p, K const& k, V const& v, json::tag_end) const
    {
        return result_type();
    }
    template <typename T, typename K, typename V>
    result_type access(T const& p, K const& k, V const& v) const
    {
        return result_type();
    }
};

} //namespace json

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

inline std::ostream& operator<<(std::ostream& out, json::object const& o)
{
    return out << json::encode(o);
}

inline std::ostream& operator<<(std::ostream& out, json::array const& o)
{
    return out << json::encode(o);
}


#endif

