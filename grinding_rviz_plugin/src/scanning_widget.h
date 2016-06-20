#ifndef SCANNING_WIDGET_H
#define SCANNING_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <scanning/ScanningService.h>
#include <publish_meshfile/PublishMeshfileService.h>
#endif

class QDoubleSpinBox;
class QWidget;
class QLineEdit;
class QPushButton;
class QTabWidget;

namespace grinding_rviz_plugin
{
class ScanningWidget : public QWidget
{

  Q_OBJECT
public:
  ScanningWidget(QWidget* parent =  NULL);

  scanning::ScanningService::Request getScanningParams();
  publish_meshfile::PublishMeshfileService::Request getPublishParams();
  void setScanningParams(scanning::ScanningService::Request params);
  void setPublishParams(publish_meshfile::PublishMeshfileService::Request params);

  void connectToServices();

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();
  void sendStatus(QString status);
  void sendMsgBox(QString title, QString msg, QString info_msg);
  void enablePanel(bool);
  void enableScanningButton();
  void enablePanelAlignment();
  void enablePanelPathPlanning();
  void enableScanWidget();
  void sendCADDatas(QString cad_path, QString cad_marker_name);
  void sendScanDatas(QString scan_path, QString scan_marker_name);

public Q_SLOTS:
  virtual void publishCADMeshOrCloudFile();
  virtual void publishScanMeshOrCloudFile();
  virtual void scanning();
  void newStatusMessage(const std_msgs::String::ConstPtr& msg);

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void browseCADFiles();
  void browseTrajectoryFiles();
  void browseCalibrationFiles();
  void importScanFileButtonHandler();
  void browseScannedFiles();
  void importCADFileButtonHandler();
  void scanningButtonHandler();
  void enableScanningButtonHandler();
  void enableScanWidgetHandler();

protected:
  const std::string package_name_;

  // ROS
  ros::NodeHandle nh_;

  ros::ServiceClient scanning_service_;
  scanning::ScanningService srv_scanning_;

  ros::ServiceClient publish_meshfile_service_;
  publish_meshfile::PublishMeshfileService srv_publish_meshfile_;

  scanning::ScanningService::Request scanning_params_;
  publish_meshfile::PublishMeshfileService::Request publish_meshfile_params_;

  // Status subscriber, allows to receive status messages from the exterior
  ros::Subscriber status_sub_;

  // GUI
  // Import CAD
  QLineEdit *cad_meshname_;
  QPushButton *cad_meshname_browse_button_;
  QPushButton *import_cad_button_;

  // Give a marker name for CAD file
  QLineEdit *cad_marker_name_line_;

  // Container for all the scan import widgets
  QWidget* scan_choice_container_;
  // Create a QTabWidget for the choice between make a scan or import a point cloud
  QTabWidget *scan_choice_widget_;
  QWidget *start_scan_tab_;
  QWidget *import_scan_tab_;

  // Widget for scan
  // YAML part in order to parse joint values
  QLineEdit *traj_yaml_file_;
  QPushButton *traj_yaml_browse_button_;
  //Parameters for SLS-2
  QLineEdit *sls_2_server_name_;
  QLineEdit *sls_2_ip_address_;
  //YAML part in order to parse calibration sls2 matrix value
  QLineEdit *calibration_yaml_file_;
  QDoubleSpinBox* down_sampling_leaf_size_;
  QPushButton *calibration_yaml_browse_button_;

  //Widget for point cloud
  QLineEdit *scan_file_;
  QPushButton *scan_file_browse_button_;
  QPushButton *import_scan_;

  // Widget for scan file
  QLineEdit *scan_marker_name_line_;

  QPushButton *start_scan_;
};

} // End namespace


#endif // SCANNING_WIDGET_H
