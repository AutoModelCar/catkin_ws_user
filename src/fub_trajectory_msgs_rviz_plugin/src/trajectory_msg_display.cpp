#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"

#include "rviz/ogre_helpers/billboard_line.h"
#include "trajectory_msg_display.h"

namespace rviz
{

TrajectoryDisplay::TrajectoryDisplay()
{
    line_width_property_ = new FloatProperty( "Line Width", 0.1,
                                            "The width, in meters, of each path line.",
                                            this, SLOT( updateLineWidth() ), this );
    line_width_property_->setMin( 0.001 );
    
    alpha_property_ = new FloatProperty( "Alpha", 1.0,
                                       "Amount of transparency to apply to the path [0,1]", this );
    alpha_property_->setMin( 0.0 );
    alpha_property_->setMax( 1.0 );
    
    color_offset_ = new FloatProperty( "Color Offset", 0.0,
                                       "Offset hue value of path trajectory color [0,1]", this );
    color_offset_->setMin( 0.0 );
    color_offset_->setMax( 1.0 );
    
    max_velocity_ = new FloatProperty( "Max Velocity", 15.0,
                                       "Value for max velocity (green trajectory) [m/s]", this );
    max_velocity_->setMin( 0.0 );
    
    buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                             "Number of paths to display.",
                                             this, SLOT( updateBufferLength() ));
    buffer_length_property_->setMin( 1 );
    
    offset_property_ = new VectorProperty( "Offset", Ogre::Vector3::ZERO,
                                         "Allows you to offset the path from the origin of the reference frame.  In meters.",
                                         this, SLOT( updateOffset() ));
}

TrajectoryDisplay::~TrajectoryDisplay()
{
    destroyObjects();
}

void TrajectoryDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateBufferLength();
}

void TrajectoryDisplay::reset()
{
    MFDClass::reset();
    updateBufferLength();
}

void TrajectoryDisplay::updateLineWidth()
{
    float line_width = line_width_property_->getFloat();

    for ( size_t i = 0; i < billboard_lines_.size(); i++ ) {
        rviz::BillboardLine* billboard_line = billboard_lines_[ i ];
        if ( billboard_line )
            billboard_line->setLineWidth( line_width );
    }
    context_->queueRender();
}

void TrajectoryDisplay::updateOffset()
{
    scene_node_->setPosition( offset_property_->getVector() );
    context_->queueRender();
}

void TrajectoryDisplay::destroyObjects()
{
    // Destroy all billboards, if any
    for ( size_t i = 0; i < billboard_lines_.size(); i++ ) {
        rviz::BillboardLine*& billboard_line = billboard_lines_[ i ];
        if ( billboard_line ) {
            delete billboard_line; // also destroys the corresponding scene node
            billboard_line = NULL; // ensure it doesn't get destroyed again
        }
    }
}

void TrajectoryDisplay::updateBufferLength()
{
    // Delete old path objects
    destroyObjects();

    // Read option
    int buffer_length = buffer_length_property_->getInt();
    billboard_lines_.resize( buffer_length );
    for ( size_t i = 0; i < billboard_lines_.size(); i++ ) {
      rviz::BillboardLine* billboard_line = new rviz::BillboardLine(scene_manager_, scene_node_);
      billboard_lines_[ i ] = billboard_line;
    }


}

bool validateFloats( fub_trajectory_msgs::TrajectoryConstPtr const & msg )
{
    bool valid = true;
    for (const fub_trajectory_msgs::TrajectoryPoint & point : msg->trajectory) {
        valid = valid && validateFloats( point.pose );
    }
    return valid;
}

void TrajectoryDisplay::processMessage( fub_trajectory_msgs::TrajectoryConstPtr const & msg )
{
    // Calculate index of oldest element in cyclic buffer
    size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();
    
    rviz::BillboardLine* billboard_line = NULL;

    // Delete oldest element
    billboard_line = billboard_lines_[ bufferIndex ];
    billboard_line->clear();

    // Check if path contains invalid coordinate values
    if ( !validateFloats( msg )) {
        setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
        return;
    }

    // Lookup transform into fixed frame
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if ( !context_->getFrameManager()->getTransform( msg->header, position, orientation )) {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    }

    Ogre::Matrix4 transform( orientation );
    transform.setTrans( position );

    Ogre::ColourValue color;
    color.a = alpha_property_->getFloat();
    
    uint32_t num_points = msg->trajectory.size();
    float line_width = line_width_property_->getFloat();

    billboard_line->setNumLines( 1 );
    billboard_line->setMaxPointsPerLine( num_points );
    billboard_line->setLineWidth( line_width );

    for (const fub_trajectory_msgs::TrajectoryPoint & point : msg->trajectory) {
        const geometry_msgs::Point& pos = point.pose.position;
        Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
        tf::Vector3 lin_vel;
        tf::vector3MsgToTF(point.velocity.linear,lin_vel);
        float abs_vel = lin_vel.length();
        float scaled_vel = fmin(abs_vel/max_velocity_->getFloat(),1.0);
        
        color.setHSB((0.25*scaled_vel) + color_offset_->getFloat(),1,1);
        billboard_line->addPoint( xpos, color );
    }

}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::TrajectoryDisplay, rviz::Display )
