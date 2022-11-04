/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreRay.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <OgreCamera.h>

#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"
#include "rviz/geometry.h"
#include "laser_pointer_init.h"

#include <sstream>
namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class Arrow;
class DisplayContext;
}

namespace rviz_plugin_tutorials
{

laser_pointer_init::laser_pointer_init()
  : Tool()
  , arrow_( NULL )
{
}

laser_pointer_init::~laser_pointer_init()
{
    delete arrow_;
}

void laser_pointer_init::onInitialize()
{
  std_cursor_ = rviz::getDefaultCursor();
  hit_cursor_ = rviz::makeIconCursor( "package://rviz/icons/crosshair.svg" );
  arrow_ = new rviz::Arrow( scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f );
  arrow_->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
  arrow_->getSceneNode()->setVisible( false );
  state_ = Position;
  topic_property_ = new rviz::StringProperty( "Topic", "anchor_point",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );

}

void laser_pointer_init::activate()
{

}

void laser_pointer_init::deactivate()
{
  
}

int laser_pointer_init::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = 0;
  static Ogre::Vector3 last_pos;
  Ogre::Vector3 pos;
  Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );
  bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
  setCursor( success ? hit_cursor_ : std_cursor_ );
  if(state_==Position){
    if ( !success )
    {
      Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
          (float)event.x / (float)event.viewport->getActualWidth(),
          (float)event.y / (float)event.viewport->getActualHeight() );

      pos = mouse_ray.getPoint(1.0);
      setStatus( "<b>Left-Click:</b> Look in this direction." );
    }
    else
    {
      std::ostringstream s;
      s << "<b>Left-Click:</b> Focus on this point.";
      s.precision(3);
      s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
      setStatus( s.str().c_str() );
    }
    if( event.leftDown() )
    {
      arrow_->setPosition( pos );
      state_ = Orientation;
      end_pos=pos;
      last_pos=pos;
    }

  }
  else{
      if(event.type == QEvent::MouseMove)
      {
        //compute angle in x-y plane
        Ogre::Vector3 cur_pos;
        Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
        if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                        ground_plane,
                                        event.x, event.y, cur_pos ))
        {
          end_yaw = atan2( cur_pos.y - last_pos.y, cur_pos.x - last_pos.x );
          arrow_->getSceneNode()->setVisible( true );
          arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian(end_yaw), Ogre::Vector3::UNIT_Z ) * orient_x );

        }
      }
      else if(event.leftUp()){
        ROS_WARN("3D Goal Set");
        std::string fixed_frame = context_->getFixedFrame().toStdString();
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, end_yaw);
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(end_pos.x, end_pos.y, end_pos.z)), ros::Time::now(), fixed_frame);
        geometry_msgs::PoseStamped goal;
        tf::poseStampedTFToMsg(p, goal);
        ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, end_yaw);
        pub_.publish(goal);
        flags |= Finished;
        state_=Position;
        arrow_->getSceneNode()->setVisible( false );
        return flags;
      }
  }

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::laser_pointer_init, rviz::Tool )