#ifndef GRID_LAYER_H
#define GRID_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace other_layer_namespace {

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {  // extends the Costmap2D class
public:
    GridLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    
    // Used for resizing (true)
    bool isDiscretized() 
    {
        return true;
    }

    virtual void matchSize();

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

};
} //end namespace
#endif