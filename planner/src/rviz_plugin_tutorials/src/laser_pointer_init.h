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

#ifndef LASER_POINTER_INIT_H
#define LASER_POINTER_INIT_H

#include "rviz/tool.h"
#include "rviz/ogre_helpers/arrow.h"
#include <QCursor>
#include <tf/transform_listener.h>
#include "rviz/geometry.h"
#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class Arrow;
class DisplayContext;
class StringProperty;
}
namespace rviz_plugin_tutorials
{
//! The Focus Tool allows the user to set the focal point of the current
//! view controller with a single mouse click.
class laser_pointer_init : public rviz::Tool
{
Q_OBJECT
public:
  laser_pointer_init();
  ~laser_pointer_init() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;
  rviz::Arrow* arrow_;
  enum State
  {
    Position,
    Orientation,
  };
  State state_;
  Ogre::Vector3 end_pos;
  double end_yaw;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
  void updateTopic();
};

} // namespace rviz

#endif
