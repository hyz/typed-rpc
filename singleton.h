#ifndef SINGLETON_H__
#define SINGLETON_H__

#include <boost/noncopyable.hpp>
#include <boost/assert.hpp>

template <class T>
struct singleton : boost::noncopyable
{
    static T& instance();

protected:
    singleton()
    {
        BOOST_ASSERT(!s_this_);
        if (s_this_) {
            std::cerr << "this is singleton!\n";
            abort();
        }
        s_this_ = static_cast<T*>(this);
    }
    ~singleton() { s_this_ = 0; }

    static T* s_this_;
};
template<class T> T* singleton<T>::s_this_ = 0;

template <class T>
T& singleton<T>::instance()
{
    BOOST_ASSERT(s_this_);
    // if (!s_this_) { abort(); }
    return *s_this_;
}

#endif // SINGLETON_H__

