#ifndef PATH_PLANNING_WIDGET_H
#define PATH_PLANNING_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <path_planning/PathPlanningService.h>
#endif

class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QWidget;
class QLineEdit;
class QRadioButton;
class QPushButton;
class QMessageBox;

namespace grinding_rviz_plugin
{
class PathPlanningWidget : public QWidget
{

  Q_OBJECT
public:
  PathPlanningWidget(QWidget* parent =  NULL);

  path_planning::PathPlanningService::Request getPathPlanningParams();
  void setPathPlanningParams(path_planning::PathPlanningService::Request params);

  std::vector<geometry_msgs::Pose> getRobotPoses();
  std::vector<bool> getPointColorViz();
  std::vector<int> getIndexVector();

  void connectToServices();

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();
  void sendStatus(QString status);
  void sendMsgBox(QString title, QString msg, QString info_msg);
  void enablePanel(bool);
  void enableComputeTrajectoryButton(bool);
  void enableVizSimButton();
  void enablePanelPostProcessor();
  void getCADAndScanParams();

public Q_SLOTS:
  virtual void ComputeTrajectory();
  virtual void VisualizeTrajectory();
  virtual void SimulateTrajectory();
  void newStatusMessage(const std_msgs::String::ConstPtr& msg);

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void ComputeTrajectoryButtonHandler();
  void VisualizeTrajectoryButtonHandler();
  void SimulateTrajectoryButtonHandler();
  void enableComputeTrajectoryButtonHandler(bool);
  void enableVizSimButtonHandler();
  void generateTrajectoryButtonHandler();
  void setCADAndScanParams(const QString cad_filename,
                           const QString cad_marker_name,
                           const QString scan_filename,
                           const QString scan_marker_name);

protected:
  // ROS
  ros::NodeHandle nh_;
  ros::ServiceClient path_planning_service_;
  path_planning::PathPlanningService srv_path_planning_;

  path_planning::PathPlanningService::Request path_planning_params_;

  // Status subscriber, allows to receive status messages from the exterior
  ros::Subscriber status_sub_;

  // GUI
  QSpinBox *covering_percentage_;

  QSpinBox *extrication_frequency_;

  QSpinBox *extrication_coefficient_;
  QLabel *grind_diameter_label_;
  QDoubleSpinBox *grind_diameter_;

  QDoubleSpinBox *depth_of_pass_;

  QRadioButton *lean_angle_axis_x_;
  QRadioButton *lean_angle_axis_y_;
  QRadioButton *lean_angle_axis_z_;

  QDoubleSpinBox *angle_value_;

  QPushButton *compute_trajectory_;
  QPushButton *execute_trajectory_;
  QPushButton *visualize_trajectory_;
};

} // End namespace


#endif // PATH_PLANNING_WIDGET_H
