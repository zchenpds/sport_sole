/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

/*
 * Modification is made by Zhuo Chen in 2021 to display custom sport_sole messaages
 */

#ifndef SPORT_SOLE_DISPLAY_H
#define SPORT_SOLE_DISPLAY_H

#include <vector>

#include "sport_sole/SportSole.h"
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace sport_sole
{

class SportSoleVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// SportSoleDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Imu message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  
//
// The SportSoleDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, SportSoleVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class SportSoleDisplay: public rviz::MessageFilterDisplay<sport_sole::SportSole>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  SportSoleDisplay();
  virtual ~SportSoleDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateColorAndAlpha();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const sport_sole::SportSole::ConstPtr& msg );

  // Storage for the list of visuals. 
  std::vector<SportSoleVisual> visuals_;

  // User-editable property variables.
  boost::shared_ptr<rviz::ColorProperty> color_ff_property_; // Forefoot
  boost::shared_ptr<rviz::ColorProperty> color_mf_property_; // Midfoot
  boost::shared_ptr<rviz::ColorProperty> color_hf_property_; // Hindfoot
  boost::shared_ptr<rviz::FloatProperty> alpha_property_;

};
// END_TUTORIAL

} // end namespace sport_sole

#endif // SPORT_SOLE_DISPLAY_H
// %EndTag(FULL_SOURCE)%