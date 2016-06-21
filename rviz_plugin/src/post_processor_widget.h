#ifndef POST_PROCESSOR_WIDGET_H
#define POST_PROCESSOR_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

class QLabel;
class QHBoxLayout;
class QVBoxLayout;
class QLineEdit;
class QPushButton;
class QCheckBox;

#include <fanuc_grinding_post_processor/PostProcessorService.h>

namespace fanuc_grinding_rviz_plugin
{
class PostProcessorWidget : public QWidget
{

  Q_OBJECT
public:
  PostProcessorWidget(QWidget* parent =  NULL);

  void setPostProcessorParams(const fanuc_grinding_post_processor::PostProcessorService::Request &params);
  void setProgramLocation(const std::string &location);
  void setRobotPoses(const std::vector<geometry_msgs::Pose> &robot_poses);
  void setPointColorViz(const std::vector<bool> &point_color_viz);
  void setIndexVector(const std::vector<int> &index_vector);

  void connectToServices();

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();
  void sendStatus(QString status);
  void sendMsgBox(QString title, QString msg, QString info_msg);
  void enablePanel(bool);
  void getRobotTrajectoryData();

public Q_SLOTS:
  void generateProgram();

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void generateProgramButtonHandler();
  void tweakProgramName();
  void setIpAddressEnable(const int state);

protected:
  // ROS
  ros::NodeHandle post_processor_node_;
  ros::ServiceClient post_processor_service_;
  fanuc_grinding_post_processor::PostProcessorService srv_post_processor_;

  // GUI
  QLineEdit* program_name_;
  QLineEdit* comment_;
  QCheckBox* upload_program_;
  QLabel* ip_adress_label_;
  QLineEdit* ip_address_;
  QLineEdit* program_location_;
  QPushButton* generate_program_button_;
};

} // End namespace


#endif // POST_PROCESSOR_WIDGET_H
