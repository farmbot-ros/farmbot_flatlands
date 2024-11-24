
#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <memory>
#include <string>

// boost
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

namespace sim {
    namespace bg = boost::geometry;
    typedef bg::model::d2::point_xy<double> Point;
    typedef bg::model::polygon<Point> Polygon;
}// namespace sim

#endif // OBSTACLE_HPP
