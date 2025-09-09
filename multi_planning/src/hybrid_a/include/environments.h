#pragma once

#include "type.h"
#include "hybrid_a/state_node.h"
#include "carconfig.h"
#include "voronoi/dynamicvoronoi.h"
#include "voronoi/voronoiedge.h"
class environment
{
public:
    explicit environment(double xl, double xu, double yl, double yu,double state_scale, double map_scale, int size_phi,carconfig& _cfg);
    ~environment();
    void setObstacles(bool **binMap);
    void setObstacle(double pt_x, double pt_y);
    void setObstacle(unsigned int x, unsigned int y);

    bool hasObstacle(const int grid_idx_x, const int grid_idx_y) const;
    bool hasObstacle(const Vec2i &grid_idx) const;
    // todo 加入车辆配置检查碰撞
    bool checkCollision(const double x, const double y, const double theta) const;

    Vec3i state2index(const Vec3d &state) const;
    Vec2d index2coord(const Vec2i &index) const;
    Vec2i coord2index(const Vec2d &coord) const;

    uint64_t getIndexNum(const Vec3i& index);
    // debug
    // uint8_t *get_map_data();

public:
    bool outBoundary(const Vec2d &pt) const;
    bool lineCheck(double x0, double y0, double x1, double y1) const;

public:
    // std::shared_ptr<uint8_t *> map_data ;
    uint8_t *map_data = nullptr;
    double map_grid_resolution;
    int map_grid_sizeX, map_grid_sizeY;
    double map_xl, map_xu, map_yl, map_yu;

    double state_grid_resolution;
    double angular_resolution;
    int state_sizeX, state_sizeY, state_sizePHI;

    carconfig& cfg;

    DynamicVoronoi voronoiDiagram;
    voronoiedge ve_calculator;
    StateNode::ptr ***state_node_map;
};
