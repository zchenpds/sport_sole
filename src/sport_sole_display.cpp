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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "sport_sole/sport_sole_visual.h"
#include "sport_sole/sport_sole_display.h"
#include "sport_sole/sport_sole_common.h"

namespace sport_sole
{

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
SportSoleDisplay::SportSoleDisplay()
{
  color_ff_property_.reset(new rviz::ColorProperty( "Forefoot Color", QColor( 204, 51, 204 ),
      "Color to draw the forefoot arrows.", this, SLOT( updateColorAndAlpha() )));
  color_hf_property_.reset(new rviz::ColorProperty( "Hindfoot Color", QColor( 204, 204, 51 ),
      "Color to draw the forefoot arrows.", this, SLOT( updateColorAndAlpha() )));  


  alpha_property_.reset(new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() )));
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void SportSoleDisplay::onInitialize()
{
  MFDClass::onInitialize();
  visuals_.reserve(4);
  for (int i = 0; i < 4; ++i)
  {
    visuals_.emplace_back( context_->getSceneManager(), scene_node_ );
  }
  updateColorAndAlpha();
}

SportSoleDisplay::~SportSoleDisplay()
{
}

// Clear the visuals by deleting their objects.
void SportSoleDisplay::reset()
{
  MFDClass::reset();
}

// Set the current color and alpha values for each visual.
void SportSoleDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  for (size_t lr: {LEFT, RIGHT})
  {
    // Loop over two arrows
    for (int i = 0; i < 2; ++i)
    {
      int j = i + lr * 2;
      auto & visual = visuals_[j];
      Ogre::ColourValue color = i == 0 ? color_ff_property_->getOgreColor() : color_hf_property_->getOgreColor();
      visual.setColor( color.r, color.g, color.b, alpha );
    }
  }
}

// This is our callback to handle an incoming message.
void SportSoleDisplay::processMessage( const sport_sole::SportSole::ConstPtr& msg )
{
  for (size_t lr: {LEFT, RIGHT})
  {
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    std::string frame_id(lr == LEFT ? "sport_sole_left" : "sport_sole_right");

    if( !context_->getFrameManager()->getTransform( frame_id,
                                                    ros::Time(),
                                                    position, orientation ))
    {
      ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                frame_id.c_str(), qPrintable( fixed_frame_ ));
      return;
    }

    // Loop over 8 cells
    double pressure_ff = 0.0, pressure_hf = 0.0;
    Ogre::Vector3 cop_ff(0.0, 0.0, 0.0), cop_hf(0.0, 0.0, 0.0);
    for (int i = 0; i < 7; ++i)
    {
      int j = i + lr * 8;
      Ogre::Vector3 pos(Rho[j] * cos(Theta[j]), Rho[j] * sin(Theta[j]), 0.0f);
      double pressure = msg->pressures[j];
      if (i < 5) 
      {
        cop_ff += pos * pressure;
        pressure_ff += pressure;
      }
      else
      {
        cop_hf += pos * pressure;
        pressure_hf += pressure;
      }
      
    }
    const double MIN_PRESSURE = 0.1;
    cop_ff /= std::max(pressure_ff, MIN_PRESSURE);
    cop_hf /= std::max(pressure_hf, MIN_PRESSURE);

    // Loop over 2 arrows 
    for (int i = 0; i < 2; ++i)
    {
      int j = i + lr * 2;
      auto visual = visuals_.begin() + j;
      // Now set or update the contents of the chosen visual.
      if (i == 0) visual->setArrow( cop_ff, pressure_ff / 15000.0 );
      else visual->setArrow( cop_hf, pressure_hf / 4000.0 );
      visual->setFramePosition( position );
      visual->setFrameOrientation( orientation );
    }
  }
}

} // end namespace sport_sole

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sport_sole::SportSoleDisplay,rviz::Display )