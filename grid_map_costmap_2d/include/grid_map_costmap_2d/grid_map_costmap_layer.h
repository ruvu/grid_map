#pragma once

#include <costmap_2d/costmap_layer.h>
#include <grid_map_core/GridMap.hpp>

#include <grid_map_msgs/GridMap.h>

namespace grid_map
{
class GridMapCostmapLayer : public costmap_2d::Layer
{
public:
  GridMapCostmapLayer();
  virtual ~GridMapCostmapLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  unsigned char getCost(float value);

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void gridMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

  GridMap grid_map_;
  tf::StampedTransform transform_to_global_frame_;
  ros::Subscriber map_subscriber_;
  std::string global_frame_id_;
  std::string layer_name_;
  double obstacle_lower_value_;
};
}
