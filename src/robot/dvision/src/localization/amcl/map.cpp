#include "dvision/amcl/map.hpp"
#include "dconfig/dconstant.hpp"
#include <ros/ros.h>
#include <cmath>
#include <iostream>
using namespace dconstant;
using namespace std;


namespace dvision {

Map::Map() {}

int Map::width() {
    return m_width;
}

int Map::height() {
    return m_height;
}

int Map::maxOccDist() {
    return m_maxOccDist;
}

Cell& Map::getCell(int x, int y) {
    if(x < 0 || x >= m_width || y < 0 || y>= m_height) {
        ROS_ERROR("Map %d * %d get cell error: x: %d, y: %d", m_width, m_height, x, y);
        return m_cells[0];
    }
    return m_cells.at(y * m_width + x);
}


int Map::getDist(float x, float y) {
    auto p = fieldToMap((int)x, (int)y);

    if(p.first < 0 || p.first >= m_width || p.second < 0 || p.second >= m_height) {
        return m_maxOccDist;
    }

    return m_cells.at(p.second * m_width + p.first).occ_dist;
}


void Map::Init() {
    // TODO(MWX): gaussian
    int gridSize = geometry::lineWidth;
    int h = geometry::fieldWidth + geometry::borderStripWidth * 2;
    int w = geometry::fieldLength + geometry::borderStripWidth * 2;
    m_width = w / gridSize + 1;
    m_height = h / gridSize + 1;
    cout << "Map w: " << m_width << " h: " << m_height << endl;
    m_gridSize = geometry::lineWidth;

    m_maxOccDist = MAX_OCC_DIST / m_gridSize;

    // Init occ dist
    m_cells.resize(m_width * m_height);
    for_each(m_cells.begin(), m_cells.end(), [this](Cell& c) {
        c.occ_dist = m_maxOccDist;
    });

    // TODO(MWX): use geometry constant
    auto upleft = fieldToMap(-450, 300);
    auto lowright = fieldToMap(450, -300);
    fillRect(upleft, lowright, 0);

    auto goal1 = fieldToMap(-450, 250);
    auto goal2 = fieldToMap(-350, -250);
    fillRect(goal1, goal2, 0);

    goal1 = fieldToMap(450, -250);
    goal2 = fieldToMap(350, 250);
    fillRect(goal2, goal1, 0);

    goal1 = fieldToMap(-510, 130);
    goal2 = fieldToMap(-450, -130);
    fillRect(goal1, goal2, 0);

    goal1 = fieldToMap(510, -130);
    goal2 = fieldToMap(450, 130);
    fillRect(goal2, goal1, 0);

    auto c1 = fieldToMap(0, 300);
    auto c2 = fieldToMap(0, -300);
    fillRect(c1, c2, 0);

    auto center = fieldToMap(0, 0);
    fillCircle(center.first, center.second, 75 / 5, 0);

    for(auto& occ: m_occupied) {
        int maxOcc = m_maxOccDist;
        int curx = occ.first;
        int cury = occ.second;
        for(int dx = -maxOcc; dx <=maxOcc; ++dx) {
            for(int dy = -maxOcc; dy <= +maxOcc; ++dy) {
                int nx = curx + dx;
                int ny = cury + dy;
                if(nx < 0 || nx >= m_width || ny < 0 || ny >= m_height)
                    continue;
                double dist = sqrt(dx * dx + dy * dy);
                auto& cell = getCell(nx, ny);
                cell.occ_dist = min(cell.occ_dist, (int)dist);
            }
        }
    }
}

// Methods to init map

pair<int, int> Map::fieldToMap(int x, int y) {
    int rx = x + geometry::wholeWidth / 2;
    int ry = geometry::wholeHeight / 2 - y;
    int gridSize = geometry::lineWidth;

    //cout << "field to Map In: " << x << " " << y << " | " << rx << " " << ry;

    return make_pair(rx / gridSize, ry / gridSize);
}

std::pair<int, int> Map::mapToField(int rx, int ry) {
    int gridSize = geometry::lineWidth;
    int x = rx * gridSize  - geometry::wholeWidth / 2;
    int y = geometry::wholeHeight / 2 - ry * gridSize;

    return make_pair(x, y);

}

void Map::fillRect(std::pair<int, int> upperLeft, std::pair<int, int> lowerRight, int dist) {
    int width = (lowerRight.first - upperLeft.first) + 1;
    int height = (lowerRight.second - upperLeft.second) + 1;
    fillRect(upperLeft.first, upperLeft.second, width, height, dist);
}

void Map::fillPoint(int x, int y, int dist) {
    m_occupied.insert(make_pair(x, y));
    getCell(x, y).occ_dist = dist;
}

void Map::horizontalFill(int x, int y, int width, int dist) {
    for(int i = 0; i < width; ++i) {
        fillPoint(x + i, y, dist);
    }
}

void Map::verticalFill(int x, int y, int height, int dist) {
    for(int i = 0; i < height; ++i) {
        fillPoint(x, y + i, dist);
    }
}

void Map::fillRect(int x, int y, int width, int height, int dist) {
    horizontalFill(x, y, width, dist);

    horizontalFill(x, y + height - 1, width, dist);

    verticalFill(x, y, height, dist);

    verticalFill(x + width - 1, y, height, dist);
}

void Map::fillCircle(int x, int y, int r, int dist) {
    // FIXME(MWX): 4 she 5 ru
    for(int i = 0; i < 360; ++i) {
        int dx = r * sin(i * 180.0 / M_PI);
        int dy = r * cos(i * 180.0 / M_PI);

        fillPoint(x + dx, y + dy, dist);
    }
}



} // namespace dvision