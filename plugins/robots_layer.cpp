#include <costmap_2d/robots_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::RobotsLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_2d {

RobotsLayer::RobotsLayer(){}
RobotsLayer::~RobotsLayer(){}

void RobotsLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  drsrv = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&RobotsLayer::reconfigureCB,this,_1,_2);
  drsrv->setCallback(cb);
}

void RobotsLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                          double* max_x, double* max_y)
{
  if(enabled_) return;
  mark_x = robot_x + cos(robot_yaw);
  mark_y = robot_y + sin(robot_yaw);
  *min_x = std::min(*min_x,mark_x);
  *min_y = std::min(*min_y,mark_y);
  *max_x = std::max(*max_x,mark_x);
  *max_y = std::max(*max_y,mark_y);

}

void RobotsLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if(enabled_) return;
  unsigned int x,y;
  if(master_grid.worldToMap(mark_x,mark_y,x,y))
  {
    master_grid.setCost(x,y,LETHAL_OBSTACLE);
  }
}

void RobotsLayer::reconfigureCB(GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
} //costmap_2d
