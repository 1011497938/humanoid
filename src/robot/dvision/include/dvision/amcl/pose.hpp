#pragma once

namespace dvision {
class Pose {
public:
    Pose();
    Pose(double x, double y, double heading);
    Pose(const Pose& other);
    bool operator==(const Pose& oth) const;
    bool operator<(const Pose& oth) const;
    double& operator[](int index);
    double x() const;
    double y() const;
    double heading() const;
    double headingR() const;
    double length() const;

    void setX(double x);
    void setY(double y);
    void setHeading(double h);
    void setHeadingR(double h);
    void rotate(double degree);

    Pose getGlobal();
private:
    double m_x;
    double m_y;
    double m_heading;

};
}