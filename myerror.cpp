#if 0
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <boost/throw_exception.hpp>
#include <boost/exception/all.hpp>
#include <boost/format.hpp>
#include "log.h"
#include "myerror.h"

using namespace std;
using namespace boost;

// void mythrow(int ln, const char* fn, int ec)
// {
// #define SIZE_ 4
//     void *buffer[SIZE_];
//     int nptrs;
// 
//     nptrs = backtrace(buffer, SIZE_);
//     // backtrace_symbols_fd(buffer, nptrs, STDERR_FILENO);
// 
//     char ** strings = backtrace_symbols(buffer, nptrs);
//     if (strings)
//     {
//         auto & log = logging::logger_stream<logging::info>::instance;
//         log << "symbols: ";
//         for (int i = 0; i < nptrs; i++)
//             log << strings[i] << "\t";
//         log.commit(__LINE__,__FILE__);
//         free(strings);
//     }
// 
//     boost::throw_exception(
//         myerror(ec) << errinfo_pos(boost::make_tuple(ln,fn))
//     );
// }

#endif
