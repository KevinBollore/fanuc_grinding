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

  // Declare variable in order to store path planning service response
  // ( Robot poses msgs, point color viz & index_vector )
  std::vector<geometry_msgs::Pose> robot_poses_msg_response;
  std::vector<bool> color_points_viz_response;
  std::vector<int> index_vector_response;

  // Status subscriber, allows to receive status messages from the exterior
  ros::Subscriber status_sub_;

  // GUI
  QLabel *covering_percentage_label_;
  QSpinBox *covering_percentage_;

  QLabel *extrication_frequency_label_;
  QSpinBox *extrication_frequency_;

  QLabel *extrication_coefficient_label_;
  QSpinBox *extrication_coefficient_;

  QLabel *grind_diameter_label_;
  QSpinBox *grind_diameter_;

  QLabel *depth_label_;
  QSpinBox *depth_spin_box_;

  QLabel *lean_angle_axis_label_;
  QRadioButton *lean_angle_axis_x_radio_;
  QRadioButton *lean_angle_axis_y_radio_;
  QRadioButton *lean_angle_axis_z_radio_;

  QLabel *angle_value_label_;
  QDoubleSpinBox *angle_value_spin_box_;

  QPushButton *compute_trajectory_button_;
  QPushButton *path_planning_simulation_button_;
  QPushButton *visualize_trajectory_button_;
};

} // End namespace


#endif // PATH_PLANNING_WIDGET_H
