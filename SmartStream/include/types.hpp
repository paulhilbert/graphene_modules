#ifndef _SmartStream_TYPES_HPP_
#define _SmartStream_TYPES_HPP_

#include <vector>

namespace FW {

typedef enum class set_type_ : int { walls, interior } set_type_t;
//typedef std::pair<set_type_t, std::vector<uint32_t>> request_t;
typedef std::tuple<Eigen::Vector4f, uint32_t, set_type_t, std::vector<uint32_t>> request_t;

} // FW

#endif /* _SmartStream_TYPES_HPP_ */
