#include <grid_map_costmap_2d/grid_map_costmap_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <limits>

PLUGINLIB_EXPORT_CLASS(grid_map::GridMapCostmapLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace grid_map
{

GridMapCostmapLayer::GridMapCostmapLayer()
{

}

GridMapCostmapLayer::~GridMapCostmapLayer()
{
}

void GridMapCostmapLayer::onInitialize()
{
  ros::NodeHandle local_nh("~/" + name_);
  ros::NodeHandle nh;

  global_frame_id_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  local_nh.param("map_topic", map_topic, std::string("map"));
  local_nh.param("layer_name", layer_name_, std::string("traversability"));
  local_nh.param("obstacle_lower_value", obstacle_lower_value_, 0.2);

  map_subscriber_ = nh.subscribe(map_topic, 1, &GridMapCostmapLayer::gridMapCallback, this);
}

void GridMapCostmapLayer::gridMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  if (!GridMapRosConverter::fromMessage(*msg, grid_map_))
  {
    ROS_ERROR("Failed to convert incoming grid map message to grid map!");
    return;
  }

  if (!grid_map_.exists(layer_name_))
  {
    ROS_ERROR("Layer with name %s not present in incoming grid map!", layer_name_.c_str());
    return;
  }

  ROS_DEBUG("Received map with frame id %s", grid_map_.getFrameId().c_str());
}

void GridMapCostmapLayer::activate()
{
  onInitialize();
}

void GridMapCostmapLayer::deactivate()
{
  map_subscriber_.shutdown();
}

void GridMapCostmapLayer::reset()
{
  deactivate();
  activate();
}

void GridMapCostmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                       double* max_x, double* max_y)
{  
  if (!grid_map_.exists(layer_name_))
  {
    return;
  }

  // Get latest transform available
  try
  {
    tf_->lookupTransform(global_frame_id_, grid_map_.getFrameId(), ros::Time(0), transform_to_global_frame_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Position position = grid_map_.getPosition();
  Position length = grid_map_.getLength();
  tf::Point b1(position.x() - .5 * length.x(), position.y() - .5 * length.y(), 0);
  tf::Point b2(position.x() + .5 * length.x(), position.y() + .5 * length.y(), 0);

  ROS_DEBUG_STREAM("Grid map bounds in " << grid_map_.getFrameId() << " frame: min=(" <<
                   b1.x() << ", " << b1.y() << "), max=(" << b2.x() << ", " << b2.y() << ")");

  // Transform the bounds to the costmap frame
  b1 = transform_to_global_frame_(b1);
  b2 = transform_to_global_frame_(b2);

  ROS_DEBUG_STREAM("Grid map bounds in " << global_frame_id_ << " frame: min=(" <<
                   b1.x() << ", " << b1.y() << "), max=(" << b2.x() << ", " << b2.y() << ")");

  *min_x = std::min(b1.x(), *min_x);
  *min_y = std::min(b1.y(), *min_y);
  *max_x = std::max(b1.x(), *max_x);
  *max_y = std::max(b1.y(), *max_y);

  *min_x = std::min(b2.x(), *min_x);
  *min_y = std::min(b2.y(), *min_y);
  *max_x = std::max(b2.x(), *max_x);
  *max_y = std::max(b2.y(), *max_y);
}

unsigned char GridMapCostmapLayer::getCost(float value)
{
  unsigned char cost = 0;
  if (std::isnan(value)) // NaN
  {
    cost = NO_INFORMATION;
  }
  else if (value > obstacle_lower_value_)
  {
    cost = LETHAL_OBSTACLE;
  }
  else
  {
    cost = FREE_SPACE;
  }

  return cost;
}

void GridMapCostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!grid_map_.exists(layer_name_))
  {
    return;
  }

  // Copy map data given proper transformations
  unsigned int mx, my;
  double max_value = 0;
  for (GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    Position position;
    grid_map_.getPosition(*it, position);
    float value = grid_map_.at(layer_name_, *it);
    tf::Point p(position.x(), position.y(), 0);

    p = transform_to_global_frame_(p);
    if (master_grid.worldToMap(p.x(), p.y(), mx, my))
    {
      if (!std::isnan(value) && value > max_value)
      {
        max_value = value;
      }
      master_grid.setCost(mx, my, getCost(value));
    }
  }
  ROS_DEBUG("Max value: %.4f", max_value);
}

}
