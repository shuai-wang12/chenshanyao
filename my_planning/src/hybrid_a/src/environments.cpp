#include "environments.h"
#include "timer.h"
#include "math/box2d.h"

using math::Box2d;

environment::~environment()
{
}

environment::environment(double xl, double xu, double yl, double yu, double state_scale, double map_scale, int size_phi,carconfig& _cfg)
    : map_xl(xl), map_xu(xu), map_yl(yl), map_yu(yu),cfg(_cfg)
{
    state_grid_resolution=state_scale;
    angular_resolution = 360.0 / size_phi * M_PI / 180.0;
    map_grid_resolution = map_scale;
    state_sizePHI = size_phi;
    state_sizeX = std::floor((map_xu - map_xl) / state_grid_resolution);
    state_sizeY = std::floor((map_yu - map_yl) / state_grid_resolution);
    map_grid_sizeX = std::floor((map_xu - map_xl) / map_grid_resolution);
    map_grid_sizeY = std::floor((map_yu - map_yl) / map_grid_resolution);

    map_data = new uint8_t[map_grid_sizeX * map_grid_sizeY];
    state_node_map=new StateNode::ptr **[state_sizeX];
    for(int i=0;i<state_sizeX;i++){
        state_node_map[i]=new StateNode::ptr *[state_sizeY];
        for(int j=0;j<state_sizeY;j++){
            state_node_map[i][j]=new StateNode::ptr [state_sizePHI];
            for(int k=0;k<state_sizePHI;k++){
                state_node_map[i][j][k]=nullptr;
            }
        }
    }
}


void environment::setObstacles(bool **binMap){
    for (unsigned int i = 0; i < map_grid_sizeX; i++)
    {
        for (unsigned int j = 0; j < map_grid_sizeY; j++)
        {
            auto x =
                static_cast<unsigned int>((i + 0.5) * map_grid_resolution /
                                        state_grid_resolution);
            auto y =
                static_cast<unsigned int>((j + 0.5) * map_grid_resolution /
                                        state_grid_resolution);

            if (binMap[x][y]) 
                setObstacle(i, j);
        }
    }
    voronoiDiagram.initializeMap(state_sizeX, state_sizeY, binMap);
    voronoiDiagram.update();
    voronoiDiagram.visualize();
    ve_calculator.initializeMap(state_sizeX, state_sizeY);
    ve_calculator.calVEdgeDist(binMap);
    ve_calculator.visualize();
}

bool environment::checkStaticCollision(const double x,const double y,math::AABox2d &rect) const{
    rect.Shift({x, y});
    Box2d box(rect);
    Vec2i transformed_pt_index_0 =
        coord2index(Vec2d(box.min_x(), box.max_y()));
    Vec2i transformed_pt_index_1 =
        coord2index(Vec2d(box.max_x(), box.max_y()));

    Vec2i transformed_pt_index_2 =
        coord2index(Vec2d(box.max_x(), box.min_y()));

    Vec2i transformed_pt_index_3 =
        coord2index(Vec2d(box.min_x(), box.min_y()));

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!lineCheck(x1, y1, x0, y0))
    {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!lineCheck(x1, y1, x0, y0))
    {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!lineCheck(x1, y1, x0, y0))
    {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!lineCheck(x0, y0, x1, y1))
    {
        return false;
    }

    return true;
}

bool environment::checkCollision(const double x, const double y,
                                 const double theta) const
{
    // Timer timer;
    Mat2d R;
    R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape_;
    transformed_vehicle_shape_.resize(8, 1);
    for (int i = 0; i < 4; i++)
    {
        transformed_vehicle_shape_.block<2, 1>(i * 2, 0) =
            R * cfg.vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    Vec2i transformed_pt_index_0 =
        coord2index(transformed_vehicle_shape_.block<2, 1>(0, 0));
    Vec2i transformed_pt_index_1 =
        coord2index(transformed_vehicle_shape_.block<2, 1>(2, 0));

    Vec2i transformed_pt_index_2 =
        coord2index(transformed_vehicle_shape_.block<2, 1>(4, 0));

    Vec2i transformed_pt_index_3 =
        coord2index(transformed_vehicle_shape_.block<2, 1>(6, 0));

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!lineCheck(x1, y1, x0, y0))
    {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!lineCheck(x1, y1, x0, y0))
    {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!lineCheck(x1, y1, x0, y0))
    {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!lineCheck(x0, y0, x1, y1))
    {
        return false;
    }

    //   check_collision_use_time += timer.End();
    //   num_check_collision++;
    return true;
}

bool environment::lineCheck(double x0, double y0, double x1, double y1) const
{
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep)
    {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1)
    {
        y_step = 1;
    }
    else
    {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i)
    {
        if (steep)
        {
            if (hasObstacle(Vec2i(yk, x0 + i * 1.0)) ||
                outBoundary(Vec2d(yk * map_grid_resolution,
                                  (x0 + i) * map_grid_resolution)))
            {
                return false;
            }
        }
        else
        {
            if (hasObstacle(Vec2i(x0 + i * 1.0, yk)) ||
                outBoundary(Vec2d((x0 + i) * map_grid_resolution,
                                  yk * map_grid_resolution)))
            {
                return false;
            }
        }

        error += delta_error;
        if (error >= 0.5)
        {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool environment::outBoundary(const Vec2d &pt) const
{
    return pt.x() < map_xl || pt.x() > map_xu ||
           pt.y() < map_yl || pt.y() > map_yu;
}

void environment::setObstacle(unsigned int x, unsigned int y)
{
    if (x < 0u || x > static_cast<unsigned int>(map_grid_sizeX) ||
        y < 0u || y > static_cast<unsigned int>(map_grid_sizeY))
    {
        return;
    }
    map_data[x + y * map_grid_sizeX] = 1;
}

void environment::setObstacle(double pt_x, double pt_y)
{
    if (pt_x < map_xl || pt_x > map_xu || pt_y < map_yl || pt_y > map_yu)
    {
        return;
    }
    int idx_x = static_cast<int>((pt_x - map_xl) / map_grid_resolution);
    int idx_y = static_cast<int>((pt_y - map_yl) / map_grid_resolution);

    map_data[idx_x + idx_y * map_grid_sizeX] = 1;
}

bool environment::hasObstacle(const Vec2i &grid_idx) const
{
    return hasObstacle(grid_idx[0], grid_idx[1]);
}

bool environment::hasObstacle(const int grid_idx_x, const int grid_idx_y) const
{
    return (grid_idx_x >= 0 && grid_idx_x < map_grid_sizeX &&
            grid_idx_y >= 0 && grid_idx_y < map_grid_sizeY &&
            (map_data[grid_idx_y * map_grid_sizeX + grid_idx_x] == 1));
}

Vec3i environment::state2index(const Vec3d &state) const
{
    Vec3i index;

    index[0] = std::min(
        std::max(int((state[0] - map_xl) / state_grid_resolution), 0),
        state_sizeX - 1);
    index[1] = std::min(
        std::max(int((state[1] - map_yl) / state_grid_resolution), 0),
        state_sizeY - 1);
    index[2] =
        std::min(std::max(int((state[2] - (-M_PI)) / angular_resolution), 0),
                 state_sizePHI - 1);

    return index;
}



Vec2d environment::index2coord(const Vec2i &index) const
{
    Vec2d pt;
    pt.x() = ((double)index[0] + 0.5) * map_grid_resolution + map_xl;
    pt.y() = ((double)index[1] + 0.5) * map_grid_resolution + map_yl;
    return pt;
}

Vec2i environment::coord2index(const Vec2d &coord) const
{
    Vec2i index;
    index[0] = int((coord[0] - map_xl) / map_grid_resolution);
    index[1] = int((coord[1] - map_yl) / map_grid_resolution);
    return index;
}

uint64_t environment::getIndexNum(const Vec3i& index){
    return index[0] + index[1] * map_grid_sizeX+index[2]*map_grid_sizeX*map_grid_sizeY;
}

// uint8_t *environment::get_map_data()
// {
//     return map_data;
// }
