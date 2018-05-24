#pragma once
#include <vector>
#include <utility>
#include <set>
#include "dvision/parameters.hpp"

namespace dvision {
static const int MAX_OCC_DIST = 25; // 100 cm;

struct Cell {
    int occ_dist;
    Cell() : occ_dist(200) {}
};

class Map {
public:
    Map();
    void Init();

    Cell& getCell(int x, int y);

    int getDist(float x, float y);
    int width();
    int height();
    int maxOccDist();
    float getScore(int x, int y);
    float getScore(float x, float y);
    std::pair<int, int> fieldToMap(int x, int y);
    std::pair<int, int> mapToField(int x, int y);

    std::set<std::pair<int, int>> getOccpied() {
        return m_occupied;
    };

private:
    void fillPoint(int x, int y, int dist);
    void horizontalFill(int x, int y, int width, int dist);
    void verticalFill(int x, int y, int height, int dist);
    void fillRect(int x, int y, int width, int height, int dist);
    void fillCircle(int x, int y, int r, int dist);

    void fillRect(std::pair<int, int> upperLeft, std::pair<int, int> lowerRight, int dist);

private:
    int m_width;
    int m_height;
    int m_maxOccDist;
    int m_gridSize;

    std::vector<Cell> m_cells;
    std::set<std::pair<int, int>> m_occupied;
};

} // namespace dvision