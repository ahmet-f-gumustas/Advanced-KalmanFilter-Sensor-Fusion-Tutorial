#ifndef INCLUDE_EKFRL_LANDMARKS_H
#define INCLUDE_EKFRL_LANDMARKS_H

#include <vector>

class Display;

struct LandmarkData
{
    double x, y;
    int id;
    LandmarkData():x(0.0),y(0.0),id(-1){}
    LandmarkData(double xPos, double yPos):x(xPos),y(yPos),id(-1){}
    LandmarkData(double xPos, double yPos, int landmarkId):x(xPos),y(yPos),id(landmarkId){}
};

struct WallSegment
{
    double x1, y1, x2, y2;
    WallSegment():x1(0),y1(0),x2(0),y2(0){}
    WallSegment(double _x1, double _y1, double _x2, double _y2):x1(_x1),y1(_y1),x2(_x2),y2(_y2){}
};

class LandmarkMap
{
    public:

        LandmarkMap();

        void addLandmark(double x, double y);

        LandmarkData getLandmarkWithId(int id) const;
        std::vector<LandmarkData> getLandmarksWithinRange(double x, double y, double range) const;
        std::vector<LandmarkData> getLandmarks() const;
        const std::vector<WallSegment>& getWalls() const;

        void render(Display& disp) const;

    private:

        std::vector<LandmarkData> m_landmarks;
        std::vector<WallSegment> m_walls;
};

#endif  // INCLUDE_EKFRL_LANDMARKS_H
