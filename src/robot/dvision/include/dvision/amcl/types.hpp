#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include "dconfig/dconstant.hpp"
#include "dvision/line_segment.hpp"



namespace dvision {
struct Control {
    Control() : dx(0.f), dy(0.f), dt(0.f) {}
    double dx;
    double dy;
    double dt;
};

struct Measurement {
    Measurement() {}

    void addWhitePoint(cv::Point2f p) {
       whitePoints.push_back(p);
    }

    void fromLines(std::vector<LineSegment>& lines) {
        for(auto& l : lines) {
            auto p1 = l.P1;
            auto p2 = l.P2;

            if(p1.x > p2.x)
                std::swap(p1, p2);

            auto dx = p2.x - p1.x;
            auto dy = p2.y - p1.y;

            auto theta = atan2(dy, dx);
            auto k = dy / dx;
            auto step_x = fabs(5 * cos(theta));
            step_x = std::max(step_x, 0.1);

            for(auto x = p1.x; x <= p2.x && x <= dconstant::geometry::wholeWidth ; x += step_x) {
                auto y = (x - p1.x) * k + p1.y;
                if(fabs(y) > dconstant::geometry::wholeHeight / 2) {
                    break;
                }
                whitePoints.push_back(cv::Point2f(x, y));
            }
        }
    }

    std::vector<cv::Point2f> whitePoints;
    std::vector<cv::Point2f> goalPosts;
    std::vector<cv::Point2f> centerPoints;
    std::vector<cv::Point2f> penaltyPoints;
};

struct Matrix {
    Matrix() {
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                m[i][j] = 0.0;
    }
    double m[3][3];
};

} // namespace dvision