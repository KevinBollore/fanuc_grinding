#ifndef PATH_PLANNING_WIDGET_H
#define PATH_PLANNING_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <fanuc_grinding_path_planning/PathPlanningService.h>
#endif

class QLabel;
class QRadioButton;
class QPushButton;
class QMessageBox;
class QCheckBox;
class QSpinBox;
class QDoubleSpinBox;

namespace fanuc_grinding_rviz_plugin
{
class PathPlanningWidget : public QWidget
{
  Q_OBJECT
public:
  PathPlanningWidget(QWidget* parent =  NULL);
  void load(const rviz::Config& config);
  void save(rviz::Config config);
  void setPathPlanningParams(fanuc_grinding_path_planning::PathPlanningService::Request &params);
  std::vector<geometry_msgs::Pose> getRobotPoses();
  std::vector<bool> getPointColorViz();
  std::vector<int> getIndexVector();

Q_SIGNALS:
  void guiChanged();
  void sendStatus(QString status);
  void sendMsgBox(QString title, QString msg, QString info_msg);
  void enablePanel(bool);
  void enableComputeTrajectoryButton(bool);
  void enableVizSimButton();
  void enablePanelPostProcessor();
  void getCADAndScanParams();

protected Q_SLOTS:
  void connectToServices();
  void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void newStatusMessage(const std_msgs::String::ConstPtr &msg);
  void computeTrajectoryButtonHandler();
  void visualizeTrajectoryButtonHandler();
  void executeTrajectoryButtonHandler();
  void pathPlanningService();
  void enableComputeTrajectoryButtonHandler(bool);
  void enableVizSimButtonHandler();
  void generateTrajectoryButtonHandler();
  void setDepthOfPassEnable(const int state);
  void setCADAndScanParams(const QString cad_filename,
                           const QString cad_marker_name,
                           const QString scan_filename,
                           const QString scan_marker_name);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient path_planning_service_;
  fanuc_grinding_path_planning::PathPlanningService srv_path_planning_;
  ros::Subscriber status_sub_;

  QCheckBox* surfacing_mode_;
  QLabel* depth_of_pass_label_;
  QDoubleSpinBox* depth_of_pass_;
  QSpinBox* covering_percentage_;
  QSpinBox* extrication_frequency_;
  QSpinBox* extrication_coefficient_;
  QDoubleSpinBox* grind_diameter_;
  QRadioButton* lean_angle_axis_x_;
  QRadioButton* lean_angle_axis_y_;
  QRadioButton* lean_angle_axis_z_;
  QDoubleSpinBox* angle_value_;
  QPushButton* compute_trajectory_;
  QPushButton* execute_trajectory_;
  QPushButton* visualize_trajectory_;
};

} // End namespace

#endif // PATH_PLANNING_WIDGET_H
