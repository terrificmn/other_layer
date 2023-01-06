#include <other_layer/other_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(other_layer_namespace::OtherLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace other_layer_namespace {

OtherLayer::OtherLayer() {}

void OtherLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &OtherLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void OtherLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
    enabled_ = config.enabled;
}

void OtherLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                double* min_y, double* max_x, double* max_y)  {
    if (!enabled_)
        return;

    mark_x_ = robot_x + cos(robot_yaw);
    mark_y_ = robot_y + sin(robot_yaw);

    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
}

void OtherLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                int max_j) {
    if (!enabled_)
        return;

    unsigned int mx;
    unsigned int my;
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)) {
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
}

} // end namespace

