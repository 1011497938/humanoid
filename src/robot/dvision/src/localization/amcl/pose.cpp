#include "dvision/amcl/pose.hpp"
#include "dvision/utils.hpp"
#include <cmath>
using namespace std;

namespace dvision {
Pose::Pose()
  : m_x(0)
  , m_y(0)
  , m_heading(0)
{
}

Pose::Pose(double x, double y, double heading)
  : m_x(x)
  , m_y(y)
  , m_heading(heading)
{
}

Pose::Pose(const Pose& other)
  : m_x(other.x())
  , m_y(other.y())
  , m_heading(other.heading())
{
}

bool Pose::operator==(const Pose &oth) const {
    if(m_x != oth.x())
        return false;

    if(m_y != oth.y())
        return false;

    return m_heading == oth.heading();

}

bool Pose::operator<(const Pose &oth) const {
    return m_x < oth.x();
}

double& Pose::operator[](int index){
    assert(index < 2);
    if(index == 0)
        return m_x;
    if(index == 1)
        return m_y;
    // FIXME(MWX): ..
    return m_y;
}

double
Pose::x() const
{
    return m_x;
}

double
Pose::y() const
{
    return m_y;
}

double
Pose::heading() const
{
    return m_heading;
}

double
Pose::headingR() const {
    return Degree2Radian(m_heading);
}

double
Pose::length() const
{
    return sqrt(m_x * m_x + m_y * m_y);
}

void
Pose::setX(double x)
{
    m_x = x;
}

void
Pose::setY(double y)
{
    m_y = y;
}

void
Pose::setHeading(double h)
{
    m_heading = h;
}

void
Pose::setHeadingR(double h)
{
    m_heading = h / M_PI * 180.0;
}

void
Pose::rotate(double t)
{
    double s = sin(t * 180.f / M_PI);
    double c = cos(t * 180.f / M_PI);

    double tmpx = c * m_x - s * m_y;
    double tmpy = s * m_x + c * m_y;

    m_x = tmpx;
    m_y = tmpy;
}
}