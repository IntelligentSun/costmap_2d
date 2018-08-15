#ifndef COSTMAP_2D_ROBOTS_LAYER_H
#define COSTMAP_2D_ROBOTS_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
//#include

namespace costmap_2d {

class RobotsLayer : public CostmapLayer
{
public:
  RobotsLayer();
  virtual ~RobotsLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

//  virtual void activate();
//  virtual void deactivate();
//  virtual void reset();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config,uint32_t level);
  double mark_x,mark_y;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *drsrv;
};

} //costmap_2d

#endif // COSTMAP_2D_ROBOTS_LAYER_H
