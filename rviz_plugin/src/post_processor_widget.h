#ifndef POST_PROCESSOR_WIDGET_H
#define POST_PROCESSOR_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <fanuc_grinding_post_processor/PostProcessorService.h>
#endif

class QLabel;
class QLineEdit;
class QPushButton;
class QCheckBox;
class QSpinBox;

namespace fanuc_grinding_rviz_plugin
{
class PostProcessorWidget : public QWidget
{
  Q_OBJECT
public:
  PostProcessorWidget(QWidget* parent =  NULL);
  void load(const rviz::Config& config);
  void save(rviz::Config config);
  void setPostProcessorParams(const fanuc_grinding_post_processor::PostProcessorService::Request &params);
  void setProgramLocation(const std::string &location);
  void setRobotPoses(const std::vector<geometry_msgs::Pose> &robot_poses);
  void setPointColorViz(const std::vector<bool> &point_color_viz);
  void setIndexVector(const std::vector<int> &index_vector);

Q_SIGNALS:
  void guiChanged();
  void sendStatus(const QString status);
  void sendMsgBox(const QString title, const QString msg, const QString info_msg);
  void enablePanel(const bool);
  void getRobotTrajectoryData();

protected Q_SLOTS:
  void connectToServices();
  void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void generateProgramButtonHandler();
  void generateProgram();
  void tweakProgramName();
  void setIpAddressEnable(const int state);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient post_processor_service_;
  fanuc_grinding_post_processor::PostProcessorService srv_post_processor_;

  QLineEdit* program_name_;
  QLineEdit* comment_;
  QSpinBox* machining_speed_;
  QSpinBox* extrication_speed_;
  QSpinBox* trajectory_z_offset_;
  QCheckBox* upload_program_;
  QLabel* ip_adress_label_;
  QLineEdit* ip_address_;
  QLineEdit* program_location_;
  QPushButton* generate_program_button_;
};

} // End namespace

#endif // POST_PROCESSOR_WIDGET_H
