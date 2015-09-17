namespace FW {

template <typename T>
inline std::set<T>
merge(const std::set<T>& set_a, const std::set<T>& set_b) {
    std::set<T> result;
    std::set_union(set_a.begin(), set_a.end(), set_b.begin(), set_b.end(),
                   std::inserter(result, result.end()));
    return result;
}

template <typename T>
inline std::set<T>
intersect(const std::set<T>& set_a, const std::set<T>& set_b) {
    std::set<T> result;
    std::set_intersection(set_a.begin(), set_a.end(), set_b.begin(),
                          set_b.end(), std::inserter(result, result.end()));
    return result;
}

template <typename T>
inline std::set<T>
subtract(const std::set<T>& set_a, const std::set<T>& set_b) {
    std::set<T> result;
    std::set_difference(set_a.begin(), set_a.end(), set_b.begin(), set_b.end(),
                        std::inserter(result, result.end()));
    return result;
}

} // FW
