#ifndef POST_PROCESSOR_WIDGET_H
#define POST_PROCESSOR_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

class QDoubleSpinBox;
class QLabel;
class QWidget;
class QHBoxLayout;
class QVBoxLayout;
class QLineEdit;
class QPushButton;

#include <post_processor/PostProcessorService.h>

namespace grinding_rviz_plugin
{
class PostProcessorWidget : public QWidget
{

  Q_OBJECT
public:
  PostProcessorWidget(QWidget* parent =  NULL);

  post_processor::PostProcessorService::Request getPostProcessorParams();
  void setPostProcessorParams(post_processor::PostProcessorService::Request params);

  void setRobotPoses(std::vector<geometry_msgs::Pose> robot_poses);
  void setPointColorViz(std::vector<bool> point_color_viz);
  void setIndexVector(std::vector<int> index_vector);

  void connectToServices();

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();
  void sendStatus(QString status);
  void enablePanel(bool);
  void getRobotPosesData();

public Q_SLOTS:
  void GenerateProgram();

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void browseProgramLocation();
  void GenerateProgramButtonHandler();
  void tweakProgramName();

protected:
  // ROS
  ros::NodeHandle post_processor_node_;
  ros::ServiceClient post_processor_service_;
  post_processor::PostProcessorService srv_post_processor_;

  post_processor::PostProcessorService::Request post_processor_params_;

  std::vector<geometry_msgs::Pose> robot_poses_;

  // GUI
  QLabel *program_name_label_;
  QLineEdit *program_name_;
  QHBoxLayout *program_name_layout_;

  QLabel *ip_adress_label_;
  QLineEdit *ip_adress_;
  QHBoxLayout *ip_adress_layout_;

  QLabel *program_location_label_;
  QPushButton *browse_program_location_button_;
  QHBoxLayout *program_location_layout_;
  QLineEdit *program_location_;

  QLabel *comment_label_;
  QLineEdit *comment_;
  QHBoxLayout *comment_layout_;

  QPushButton *generate_program_button_;

  QVBoxLayout *post_processor_layout_;
};

} // End namespace


#endif // POST_PROCESSOR_WIDGET_H
