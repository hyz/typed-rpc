#include <time.h>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <boost/system/error_code.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>

namespace multi_index = boost::multi_index;

struct resolver : tcp::resolver
{
    typedef resolver This;

    template <typename H>
    void async_query(std::string host, H&& h)
    {
        return async_query(host, false, std::forward<H>(h));
    }

    template <typename H>
    void async_query(std::string host, bool ignore_cache, H&& h)
    {
        if (ignore_cache) {
            auto p = cbs_.emplace(host, std::vector<callback_t>());
            p.first->second.push_back(h);
            resolv_(host);
            return;
        }

        auto it = resolved_.find(host);
        if (it != resolved_.end()) {
            it->tp_queryed_ = time(0);

            // relocate it;

            auto addr = it->addr;
            get_io_service().post([h,addr](){
                        h(boost::system::error_code(), addr);
                    });

            if (it->tp_resolved_ + 60*60 < time(0)) {
                return;
            }
        }

        auto p = cbs_.emplace(host, std::vector<callback_t>());
        p.first->second.push_back(h);
        resolv_(host);
    }

    void resolv_(std::string& host)
    {
        tcp::resolver::query q(host, "");
        async_resolve(q, [this,host](boost::system::error_code ec, tcp::resolver::iterator r_it){
                    handle_resolv(host, ec, r_it);
                });
    }

    void _check_time(time_t tpc)
    {
        auto& idx = boost::get<1>(resolved_);
        while (!idx.empty()) {
            auto& x = idx.front();
            if (x.tp_queryed_ + 60*60*4 < tpc) {
                break;
            }
            idx.pop_front();
        }
    }

    resolver(boost::asio::io_service& io_s)
        : tcp::resolver(io_s)
        , timer(io_s)
    {}

////// //////
    void handle_resolv(std::string host, boost::system::error_code ec, tcp::resolver::iterator it_r)
    {
        ip::address addr;
        if (!ec) {
            tcp::endpoint a = *it_r;
            addr = a.address();

            auto p = resolved_.emplace(host, addr);
            if (!p.second) {
                resolved_.modify(p.first, [&addr](record& rec){
                            rec.addr = addr;
                            rec.tp_resolved_ = time(0);
                        });
            }
        }
        auto it = cbs_.find(host);
        if (it != cbs_.end()) {
            for (auto& fn : it->second) {
                fn(ec, addr);
            }
            cbs_.erase(it);
        }
    }

    struct record
    {
        std::string host; // tcp::resolver::query query; //(args.host_, args.port_));
        ip::address addr; // tcp::endpoint endpx;
        time_t tp_resolved_;
        time_t tp_queryed_;
        record(std::string h, ip::address a)
            : host(h), addr(a)
        {
            tp_resolved_ = tp_queryed_ = time(0);
        }
    };

    boost::multi_index_container<
        record,
        multi_index::indexed_by<
            multi_index::hashed_unique<multi_index::member<record, std::string, &record::host>>
          , multi_index::sequenced<>
        >
    > resolved_;

    typedef std::function<void(boost::system::error_code, ip::address const&)> callback_t;
    std::unordered_map<std::string,std::vector<callback_t>> cbs_;

    boost::asio::deadline_timer timer;
};


