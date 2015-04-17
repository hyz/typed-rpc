
namespace variadic {
    template<typename Archive, typename... T>
    void serialize(Archive & ar, T&&... t)
    {
        using swallow = int[];
        (void)swallow{0, (void(ar & t), 0)...};
    }

    namespace detail {
        template <typename> struct serialize_helper;
        template <template<int...> class seq, int... Indices>
        struct serialize_helper<seq<Indices...>> {
            template <typename A, typename T> static void ssf(A & ar, T& t) {
                serialize(ar, std::get<Indices>(t)...);
            }
        };
    } // namespace detail

    template<typename Archive, typename Tuple>
    void serialize_tuple_unpack(Archive& ar, Tuple & tux)
    {
        detail::serialize_helper<typename ::detail::gens<std::tuple_size<Tuple>::value>::type>::template ssf(ar, tux);
    }
} // namespace variadic
