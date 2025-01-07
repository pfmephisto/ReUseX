#pragma once
#include <Eigen/Dense>
#include "types/Point.hh"
#include "types/Direction.hh"
#include "types/Vector.hh"


namespace linkml {

    static Point compute_average(const std::vector<Point>& points) {
        if (points.empty()) {
                throw std::runtime_error("Cannot compute average of an empty vector of points.");
        }

        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

        for (const auto& point : points) {
                sum_x += point.x();
                sum_y += point.y();
                sum_z += point.z();
        }

        size_t n = points.size();
        return Point(sum_x / n, sum_y / n, sum_z / n);
}

    static std::tuple<Eigen::Matrix3f, Point> get_csystem(std::vector<Point> const& points, Plane const& plane) {
        Point center = compute_average(points);
        Point point =  *points.begin();

        Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();

        Direction aX = Direction(point-center);
        Direction aY = Direction(CGAL::cross_product(plane.orthogonal_vector(), Vector(aX.dx(), aX.dy(), aX.dz())));

        Direction normal = Direction(plane.orthogonal_vector());

        mat.col(0) = Eigen::Vector3f(aX.dx(), aX.dy(), aX.dz());
        mat.col(1) = Eigen::Vector3f(aY.dx(), aY.dy(), aY.dz());
        mat.col(2) = Eigen::Vector3f(normal.dx(), normal.dy(), normal.dz());

        return std::tuple(mat, center); 
    }
}