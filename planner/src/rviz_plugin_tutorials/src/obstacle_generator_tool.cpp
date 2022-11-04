#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
// #include <OGRE/Rectangle2D.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "rviz/geometry.h"
#include "OGRE/OgreCamera.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <QVariant>
#include <visualization_msgs/Marker.h>
#include "math.h"
// #include "selected_points_publisher.hpp"

#include "obstacle_generator_tool.h"

namespace rviz_plugin_tutorials
{

ObstacleGeneratorTool::ObstacleGeneratorTool()
{
  shortcut_key_ = 'l';
}

ObstacleGeneratorTool::~ObstacleGeneratorTool()
{

}

void ObstacleGeneratorTool::onInitialize()
{

  mouse_pub = nh_.advertise<rviz_plugin_tutorials::Obstacle>("/obstacle_generator/mouse", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  line_list.header.frame_id = "world";
  line_list.lifetime = ros::Duration(500);
  line_list.ns = "lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.1;
 
  line_list.color.b = 1.0;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;
}

void ObstacleGeneratorTool::activate()
{

}

void ObstacleGeneratorTool::deactivate()
{

}

bool click_flag = false;
bool click_flag1 = false;
bool second_click_flag= false;
Ogre::Vector3 intersection1_before,intersection2_before,intersection,intersection1,intersection2;
geometry_msgs::Point p0;
geometry_msgs::Point p1;
geometry_msgs::Point p2;
geometry_msgs::Point p3;
geometry_msgs::Point p4;
geometry_msgs::Point p5;
geometry_msgs::Point p6;
geometry_msgs::Point p7;
geometry_msgs::Point p8;
geometry_msgs::Point p_ground1,p_ground2;
geometry_msgs::Point point5;
geometry_msgs::Point point6;
geometry_msgs::Point point7;
geometry_msgs::Point point8;
double length;
double alpha;
const double beta = M_PI/12;
int rotate_times = 0;
double pi=0;//劈一半
int ObstacleGeneratorTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  const double z_scale = 50;
  static double prevz;

  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    if(second_click_flag==false)
    {
                if(click_flag && !click_flag1)
          {
            intersection1.x=intersection1_before.x;
            intersection1.y=intersection1_before.y;
            double z = event.y;
            double dz = z - prevz;
            prevz = z;
            intersection1.z -= dz / z_scale;
            makeBox(intersection1_before,intersection1);//画线ing
          }
          if( event.leftDown() && !click_flag)
          {
            intersection1_before = intersection;
            prevz = event.y;
            std::cout<<"intersection1_before   "<<intersection1_before<<std::endl;
            click_flag = true;
          }
          else if( event.leftUp() && !click_flag1)//
          {
            std::cout<<"finish intersection1   "<<intersection1<<std::endl;
            click_flag1 = false;
            click_flag = false; 
            second_click_flag=true;
          }
          else if( event.rightDown() )
          {
            return Render | Finished; // Cancel to drop obstacle
          }
    }
    else{
                if(click_flag && !click_flag1)
          {
            intersection2.x=intersection2_before.x;
            intersection2.y=intersection2_before.y;
            double z = event.y;
            double dz = z - prevz;
            prevz = z;
            intersection2.z -= dz / z_scale;
            makeBox(intersection2_before,intersection2);//画线ing
          }

          if( event.leftDown() && !click_flag)
          {
            intersection2_before = intersection;
            intersection2_before.z=intersection1.z;
            prevz = event.y;
            std::cout<<"intersection2_before   "<<intersection2_before<<std::endl;
            click_flag = true;
          }
          else if( event.leftUp() && !click_flag1)
          {
            std::cout<<"finish intersection2   "<<intersection2<<std::endl;

            point7.x = intersection1[0];
            point7.y = intersection1[1];
            point7.z = 0;
            point6.x = intersection1[0];
            point6.y = intersection2[1];
            point6.z = 0;
            point5.x = intersection2[0];
            point5.y = intersection2[1];
            point5.z = 0;
            point8.x = intersection2[0];
            point8.y = intersection1[1];
            point8.z = 0;
            std::cout<<"save_point_for_rotation_finished   "<<std::endl;

            click_flag1 = false;
            click_flag = false; 
            second_click_flag=false;

          }
          else if( event.rightDown() )
          {
            return Render | Finished; // Cancel to drop obstacle
          }

    }

  }
  else
  {
   
  }
  return Render;
}
int ObstacleGeneratorTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  if (event->type() == QKeyEvent::KeyPress)
    {
      if (event->key() == 'R' || event->key() == 'r')
      {
        rotate_times = rotate_times + 1;
        std::cout<<"rotate box"<<std::endl;
        makeBoxRotation(point7, point6, point5, point8);  
      }
      else if (event->key() == 'q' || event->key() == 'Q')
      {
            std::cout<<"quit"<<std::endl;
            line_list.points.clear();
            marker_pub.publish(line_list);
            click_flag = false;
            click_flag1 = false;
            second_click_flag=0;
            rotate_times=0;
            pi=0;
            std::cout<<"clean"<<std::endl;
      }
      else if (event->key() == 'A' || event->key() == 'a')
      {
        std::cout<<"save_add"<<std::endl;
        makeObstacle( intersection1, intersection2 );
        click_flag = false;
        click_flag1 = false;
        line_list.points.clear();
        second_click_flag=0;
        rotate_times=0;
        pi=0;
        marker_pub.publish(line_list);
        return Render | Finished;
      }
      else if (event->key() == 'N' || event->key() == 'n')
      {
        pi=1;
        std::cout<<"劈一半"<<std::endl;
      }
    }
  return Render;
}

void ObstacleGeneratorTool::makeObstacle( const Ogre::Vector3& pos1, const Ogre::Vector3& pos2 )
{

  current_time = ros::Time::now();
  mouse_pointor.header.stamp=current_time;
  mouse_pointor.header.frame_id="world";
  mouse_pointor_pos.pose.x = (pos1[0]+pos2[0])/2;
  mouse_pointor_pos.pose.y = (pos1[1]+pos2[1])/2;
  mouse_pointor_pos.pose.z = (pos1[2]+pos2[2])/2;
  mouse_pointor_pos.shape.x = abs(pos1[0]-pos2[0]);
  mouse_pointor_pos.shape.y = abs(pos1[1]-pos2[1]);
  mouse_pointor_pos.shape.z = abs(pos1[2]-pos2[2]);
  mouse_pointor_pos.angle.x = rotate_times * beta;
  mouse_pointor_pos.angle.y = pi;
  std::cout<<"mouse_pointor_pos   "<<mouse_pointor_pos<<std::endl;
  mouse_pointor_pos.header.stamp=current_time;
  mouse_pointor_pos.header.frame_id="world";
  mouse_pointor.poses.push_back(mouse_pointor_pos);
  mouse_pub.publish(mouse_pointor);
  // highlight_rectangle_->setCorners(pos1[0], pos1[1], pos2[0], pos2[1]);
}

void ObstacleGeneratorTool::makeBox(const Ogre::Vector3& pt1, const Ogre::Vector3& pt2)
{
  if(second_click_flag==false){
      line_list.points.clear();
        p1.x = pt1[0];
        p1.y = pt1[1];
        p1.z = pt1[2];
        p2.x = pt2[0];
        p2.y = pt2[1];
        p2.z = pt2[2];
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        marker_pub.publish(line_list);
  }
  else{
        line_list.points.clear();
        p_ground1.x = intersection1_before[0];
        p_ground1.y = intersection1_before[1];
        p_ground1.z = 0;

        p_ground2.x = intersection2_before[0];
        p_ground2.y = intersection2_before[1];
        p_ground2.z = 0;

        p7.x = intersection1[0];
        p7.y = intersection1[1];
        p7.z = intersection1[2];

        p4.x = intersection2[0];
        p4.y = intersection2[1];
        p4.z = intersection2[2];

        p5.x = intersection2_before[0];
        p5.y = intersection2_before[1];
        p5.z = intersection2_before[2];
        
        p8.x = intersection2[0];
        p8.y = intersection1[1];
        p8.z = intersection1[2];

        p3.x = intersection2[0];
        p3.y = intersection1[1];
        p3.z = intersection2[2];

        p1.x = intersection1[0];
        p1.y = intersection2[1];
        p1.z = intersection2[2];

        p2.x = intersection1[0];
        p2.y = intersection1[1];
        p2.z = intersection2[2];

        p6.x = intersection1[0];
        p6.y = intersection2[1];
        p6.z = intersection1[2];


        line_list.points.push_back(p1);
        line_list.points.push_back(p2);

        line_list.points.push_back(p2);
        line_list.points.push_back(p3);

        line_list.points.push_back(p3);
        line_list.points.push_back(p4);

        line_list.points.push_back(p4);
        line_list.points.push_back(p1);

        line_list.points.push_back(p1);
        line_list.points.push_back(p3);

        line_list.points.push_back(p3);
        line_list.points.push_back(p8);

        line_list.points.push_back(p8);
        line_list.points.push_back(p6);

        line_list.points.push_back(p6);
        line_list.points.push_back(p1);

        line_list.points.push_back(p2);
        line_list.points.push_back(p7);

        line_list.points.push_back(p7);
        line_list.points.push_back(p8);

        line_list.points.push_back(p8);
        line_list.points.push_back(p5);

        line_list.points.push_back(p5);
        line_list.points.push_back(p6);

        line_list.points.push_back(p6);
        line_list.points.push_back(p7);

        line_list.points.push_back(p7);
        line_list.points.push_back(p5);

        line_list.points.push_back(p5);
        line_list.points.push_back(p4);

        line_list.points.push_back(p4);
        line_list.points.push_back(p2);


        line_list.points.push_back(p7);
        line_list.points.push_back(p_ground1);

        line_list.points.push_back(p5);
        line_list.points.push_back(p_ground2);

        marker_pub.publish(line_list);
  }
 
}

void ObstacleGeneratorTool::makeBoxRotation(const geometry_msgs::Point& pt7_, const geometry_msgs::Point& pt6_, const geometry_msgs::Point& pt5_, const geometry_msgs::Point& pt8_)
{
  line_list.points.clear();
  p0.x = (pt7_.x+pt5_.x)/2;
  p0.y = (pt7_.y+pt5_.y)/2;
  p0.z = intersection1[2];
  
  p7.x = (pt7_.x - p0.x)*cos(beta) - (pt7_.y - p0.y) * sin(beta) + p0.x;
  p7.y = (pt7_.y - p0.y)*cos(beta) + (pt7_.x - p0.x) * sin(beta) + p0.y;
  p7.z = intersection1[2];

  p6.x = (pt6_.x - p0.x)*cos(beta) - (pt6_.y - p0.y) * sin(beta) + p0.x;
  p6.y = (pt6_.y - p0.y)*cos(beta) + (pt6_.x - p0.x) * sin(beta) + p0.y;
  p6.z = intersection1[2];

  p5.x = (pt5_.x - p0.x)*cos(beta) - (pt5_.y - p0.y) * sin(beta) + p0.x;
  p5.y = (pt5_.y - p0.y)*cos(beta) + (pt5_.x - p0.x) * sin(beta) + p0.y;
  p5.z = intersection1[2];

  p8.x = (pt8_.x - p0.x)*cos(beta) - (pt8_.y - p0.y) * sin(beta) + p0.x;
  p8.y = (pt8_.y - p0.y)*cos(beta) + (pt8_.x - p0.x) * sin(beta) + p0.y;
  p8.z = intersection1[2];

  p2=p7;p2.z=intersection2[2];
  p1=p6;p1.z=intersection2[2];
  p4=p5;p4.z=intersection2[2];
  p3=p8;p3.z=intersection2[2];
  p_ground1.x = p7.x;
  p_ground1.y = p7.y;
  p_ground1.z = 0;

  p_ground2.x = p5.x;
  p_ground2.y = p5.y;
  p_ground2.z = 0;

  point7 = p7;
  point6 = p6;
  point5 = p5;
  point8 = p8;
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);

        line_list.points.push_back(p2);
        line_list.points.push_back(p3);

        line_list.points.push_back(p3);
        line_list.points.push_back(p4);

        line_list.points.push_back(p4);
        line_list.points.push_back(p1);

        line_list.points.push_back(p1);
        line_list.points.push_back(p3);

        line_list.points.push_back(p3);
        line_list.points.push_back(p8);

        line_list.points.push_back(p8);
        line_list.points.push_back(p6);

        line_list.points.push_back(p6);
        line_list.points.push_back(p1);

        line_list.points.push_back(p2);
        line_list.points.push_back(p7);

        line_list.points.push_back(p7);
        line_list.points.push_back(p8);

        line_list.points.push_back(p8);
        line_list.points.push_back(p5);

        line_list.points.push_back(p5);
        line_list.points.push_back(p6);

        line_list.points.push_back(p6);
        line_list.points.push_back(p7);

        line_list.points.push_back(p7);
        line_list.points.push_back(p5);

        line_list.points.push_back(p5);
        line_list.points.push_back(p4);

        line_list.points.push_back(p4);
        line_list.points.push_back(p2);


        line_list.points.push_back(p7);
        line_list.points.push_back(p_ground1);

        line_list.points.push_back(p5);
        line_list.points.push_back(p_ground2);

        marker_pub.publish(line_list);

}

void ObstacleGeneratorTool::makeFlag( const Ogre::Vector3& position )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  flag_nodes_.push_back( node );
}



void ObstacleGeneratorTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our flags
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``flags_config``) to store
  // the list.
  rviz::Config flags_config = config.mapMakeChild( "Flags" );

  // To read the positions and names of the flags, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single flag and append it to the Config list.
    rviz::Config flag_config = flags_config.listAppendNew();
    // Into the flag's config we store its name:
    flag_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( flag_config );
  }
}

void ObstacleGeneratorTool::load( const rviz::Config& config )
{
  // Here we get the "Flags" sub-config from the tool config and loop over its entries:
  rviz::Config flags_config = config.mapGetChild( "Flags" );
  int num_flags = flags_config.listLength();
  for( int i = 0; i < num_flags; i++ )
  {
    rviz::Config flag_config = flags_config.listChildAt( i );
    // At this point each ``flag_config`` represents a single flag.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Flag " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``flag_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    flag_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( flag_config );
    // We finish each flag by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible flag object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeFlag( prop->getVector() );
    // makeObstacle( prop->getVector() );
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ObstacleGeneratorTool,rviz::Tool )