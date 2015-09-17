#ifndef _SmartStream_SET_ALGO_HPP_
#define _SmartStream_SET_ALGO_HPP_

#include <set>
#include <algorithm>

namespace FW {

template <typename T>
std::set<T> merge(const T& set_a, const T& set_b);

template <typename T>
std::set<T> intersect(const T& set_a, const T& set_b);

template <typename T>
std::set<T> subtract(const T& set_a, const T& set_b);

} // FW

#include "set_algo.ipp"

#endif /* _SmartStream_SET_ALGO_HPP_ */
