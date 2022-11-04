#ifndef OBSTACLE_GENERATOR_TOOL_H
#define OBSTACLE_GENERATOR_TOOL_H

#include <rviz/tool.h>
#include <ros/ros.h>
#include <QCursor>
#include <QObject>
// #include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "rviz_plugin_tutorials/Obstacle.h"
#include "rviz_plugin_tutorials/ObstaclePose.h"

namespace Ogre
{
class SceneNode;
class Vector3;
class Rectangle2D;
// class AxisAlignedBox;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class ObstacleGeneratorTool: public rviz::Tool
{
Q_OBJECT
public:
  ObstacleGeneratorTool();
  ~ObstacleGeneratorTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:
  void makeFlag( const Ogre::Vector3& position );
  void makeObstacle( const Ogre::Vector3& pos1, const Ogre::Vector3& pos2 );
  void makeBox(const Ogre::Vector3& pt1, const Ogre::Vector3& pt2);
  void makeBoxRotation(const geometry_msgs::Point& pt1_, const geometry_msgs::Point& pt2_, const geometry_msgs::Point& pt3_, const geometry_msgs::Point& pt4_);

  std::vector<Ogre::SceneNode*> flag_nodes_;
  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;
  rviz::VectorProperty* current_flag_property_;
  
  rviz_plugin_tutorials::Obstacle mouse_pointor;
  rviz_plugin_tutorials::ObstaclePose mouse_pointor_pos;
  ros::Time current_time;

  visualization_msgs::Marker line_list;

protected:
  ros::Publisher mouse_pub;
  ros::Publisher marker_pub;
  ros::NodeHandle nh_;


};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // PLANT_FLAG_TOOL_H
