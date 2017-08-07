#ifndef RVIZ_TRAJECTORY_MSG_DISPLAY_H
#define RVIZ_TRAJECTORY_MSG_DISPLAY_H

#include <fub_trajectory_msgs/Trajectory.h>

#include "rviz/message_filter_display.h"

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class BillboardLine;
class VectorProperty;


/**
 * \class TrajectoryDisplay
 * \brief Displays a nav_msgs::Path message
 */
class TrajectoryDisplay: public MessageFilterDisplay<fub_trajectory_msgs::Trajectory>
{
Q_OBJECT
public:
  TrajectoryDisplay();
  virtual ~TrajectoryDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage( fub_trajectory_msgs::TrajectoryConstPtr const & msg );

private Q_SLOTS:
  void updateBufferLength();
  void updateLineWidth();
  void updateOffset();

private:
  void destroyObjects();

  std::vector<rviz::BillboardLine*> billboard_lines_;

  FloatProperty* max_velocity_;
  FloatProperty* color_offset_;
  FloatProperty* alpha_property_;
  FloatProperty* line_width_property_;
  IntProperty* buffer_length_property_;
  VectorProperty* offset_property_;

};

} // namespace rviz

#endif /* RVIZ_PATH_DISPLAY_H */

