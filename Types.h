#ifndef LB_TYPES_H
#define LB_TYPES_H

#include <iomanip>
#include <chrono>
#include <vector>
#include "boost/multi_array.hpp"
#include <algorithm>

/** Top level namespace */
namespace lb {
#define PI 3.14159265359f

    typedef std::size_t sizet;
    typedef unsigned char uchar;


    typedef std::vector<float> vec1;
    typedef boost::multi_array<float, 2>  vec2;

    typedef boost::multi_array_types::index_range index_range;
    typedef vec2::array_view<2>::type view2;

    /** High resolution clock type */
    typedef std::chrono::high_resolution_clock timer_type;

/** High resolution clock units type */
    typedef std::chrono::duration<float, std::milli> milliseconds;

/** High resolution clock time point type */
    typedef typename timer_type::time_point time_point;

/** High resolution clock time duration type */
    typedef typename timer_type::duration duration;

} // lb

#endif // LB_TYPES_HPP_INCLUDED
