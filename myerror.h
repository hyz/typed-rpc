#ifndef _MYERROR_H_
#define _MYERROR_H_

#include <string>
#include <ostream>
//#include <fstream>
//#include <utility>
//#include <boost/tuple/tuple.hpp>
#include <boost/throw_exception.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/error_info.hpp>
#include <boost/exception/errinfo_at_line.hpp>
#include <boost/exception/errinfo_file_name.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/get_error_info.hpp>
#include <boost/system/error_code.hpp>

struct myerror
    : virtual boost::exception
    , virtual std::exception
{
    int error_code;
    myerror(int ec = 0) { error_code = ec; }
};

inline std::ostream& operator<<(std::ostream& out, myerror const& e)
    { return out << boost::diagnostic_information(e); }
inline std::ostream& operator<<(std::ostream& out, boost::exception const& e)
    { return out << boost::diagnostic_information(e); }

#endif

