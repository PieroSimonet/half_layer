#include <half_layer/half_layer.h>

#include <cmath>

using costmap_2d::LETHAL_OBSTACLE;

namespace half_layer
{

HalfLayer::HalfLayer() {}

void HalfLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &HalfLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void HalfLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void HalfLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  this->robot_yaw = robot_yaw;

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void HalfLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;  
  
  unsigned int dx;
  unsigned int dy;

  unsigned int mx;
  unsigned int my;

  unsigned int min_mx;
  unsigned int min_my; 
  unsigned int max_mx;
  unsigned int max_my;
  unsigned int my_mx;
  unsigned int my_my;

  if (master_grid.worldToMap(mark_x_, mark_y_, my_mx, my_my)) {

    if (master_grid.worldToMap(mark_x_ -5.0, mark_y_ - 5.0, min_mx, min_my)) {
      if (master_grid.worldToMap(mark_x_ + 5.0, mark_y_ + 5.0, max_mx, max_my)) {

        for (mx = min_mx; mx < max_mx; mx++) {
            for (my = min_my; my < max_my; my++) {

                int dx = (int) mx - (int) my_mx;
                int dy = (int) my - (int) my_my; 

                float angle = std::atan2(dy, dx) - robot_yaw;

                if( angle > -2.5 && angle < 2.5) {
                    try {
                        master_grid.setCost(mx , my, 0);
                    } catch (std::exception& e) {
                        ROS_ERROR("HalfLayer: %s", e.what());
                    }
                }else if (master_grid.getCost(mx, my) != costmap_2d::LETHAL_OBSTACLE){
                  // The rest of the map read it only if it is a lethel_obstacle
                  master_grid.setCost(mx, my,0);
                }
            }
        }
      }
    }
  }
}

} // end namespace
