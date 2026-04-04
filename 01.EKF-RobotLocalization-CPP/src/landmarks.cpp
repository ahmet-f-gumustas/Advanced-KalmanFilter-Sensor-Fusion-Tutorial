#include "landmarks.h"

#include <cmath>
#include "display.h"

LandmarkMap::LandmarkMap()
{
    // Indoor environment: 20m x 15m
    // Outer walls
    m_walls.push_back(WallSegment(0, 0, 20, 0));
    m_walls.push_back(WallSegment(20, 0, 20, 15));
    m_walls.push_back(WallSegment(20, 15, 0, 15));
    m_walls.push_back(WallSegment(0, 15, 0, 0));

    // Internal walls creating rooms
    // Vertical wall separating left rooms from corridor
    m_walls.push_back(WallSegment(6, 0, 6, 5.5));
    m_walls.push_back(WallSegment(6, 7, 6, 8));
    m_walls.push_back(WallSegment(6, 9.5, 6, 15));

    // Vertical wall separating corridor from right rooms
    m_walls.push_back(WallSegment(14, 0, 14, 5.5));
    m_walls.push_back(WallSegment(14, 7, 14, 8));
    m_walls.push_back(WallSegment(14, 9.5, 14, 15));

    // Horizontal wall dividing top and bottom rooms (left side)
    m_walls.push_back(WallSegment(0, 7.5, 4, 7.5));

    // Horizontal wall dividing top and bottom rooms (right side)
    m_walls.push_back(WallSegment(16, 7.5, 20, 7.5));

    // Landmarks (pillars/columns at known positions)
    // Room 1 (bottom-left): corners and center
    addLandmark(1.5, 1.5);    // 0
    addLandmark(1.5, 5.5);    // 1
    addLandmark(4.5, 1.5);    // 2
    addLandmark(4.5, 5.5);    // 3

    // Room 2 (top-left)
    addLandmark(1.5, 9.5);    // 4
    addLandmark(1.5, 13.5);   // 5
    addLandmark(4.5, 9.5);    // 6
    addLandmark(4.5, 13.5);   // 7

    // Corridor (center)
    addLandmark(10, 2.0);     // 8
    addLandmark(10, 7.5);     // 9
    addLandmark(10, 13.0);    // 10

    // Room 3 (bottom-right)
    addLandmark(15.5, 1.5);   // 11
    addLandmark(15.5, 5.5);   // 12
    addLandmark(18.5, 1.5);   // 13
    addLandmark(18.5, 5.5);   // 14

    // Room 4 (top-right)
    addLandmark(15.5, 9.5);   // 15
    addLandmark(15.5, 13.5);  // 16
    addLandmark(18.5, 9.5);   // 17
    addLandmark(18.5, 13.5);  // 18

    // Corridor entrance/exit markers
    addLandmark(7.0, 6.0);    // 19
    addLandmark(13.0, 6.0);   // 20
    addLandmark(7.0, 9.0);    // 21
    addLandmark(13.0, 9.0);   // 22
    addLandmark(10.0, 4.5);   // 23
    addLandmark(10.0, 10.5);  // 24
}

void LandmarkMap::addLandmark(double x, double y)
{
    m_landmarks.push_back(LandmarkData(x, y, m_landmarks.size()));
}

LandmarkData LandmarkMap::getLandmarkWithId(int id) const
{
    for (const LandmarkData& lm : m_landmarks){if (lm.id == id){return lm;}}
    return LandmarkData();
}

std::vector<LandmarkData> LandmarkMap::getLandmarksWithinRange(double x, double y, double range) const
{
    std::vector<LandmarkData> result;
    for (const LandmarkData& lm : m_landmarks)
    {
        double dx = lm.x - x;
        double dy = lm.y - y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < range)
        {
            result.push_back(lm);
        }
    }
    return result;
}

std::vector<LandmarkData> LandmarkMap::getLandmarks() const
{
    return m_landmarks;
}

const std::vector<WallSegment>& LandmarkMap::getWalls() const
{
    return m_walls;
}

void LandmarkMap::render(Display& disp) const
{
    // Draw walls (gray)
    disp.setDrawColour(100, 100, 100);
    for (const auto& wall : m_walls)
    {
        disp.drawLine(Vector2(wall.x1, wall.y1), Vector2(wall.x2, wall.y2));
    }

    // Draw landmarks as diamond shapes (yellow)
    double s = 0.2; // diamond half-size
    disp.setDrawColour(255, 255, 0);
    for (const auto& lm : m_landmarks)
    {
        std::vector<Vector2> diamond = {
            {lm.x, lm.y + s},
            {lm.x + s, lm.y},
            {lm.x, lm.y - s},
            {lm.x - s, lm.y},
            {lm.x, lm.y + s}
        };
        disp.drawLines(diamond);
    }
}
