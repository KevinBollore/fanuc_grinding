#ifndef SCANNING_WIDGET_H
#define SCANNING_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <fanuc_grinding_scanning/ScanningService.h>
#include <fanuc_grinding_publish_meshfile/PublishMeshfileService.h>
#endif

class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QTabWidget;

namespace fanuc_grinding_rviz_plugin
{
class ScanningWidget : public QWidget
{
  Q_OBJECT
public:
  ScanningWidget(QWidget* parent =  NULL);
  void load(const rviz::Config& config);
  void save(rviz::Config config);
  void setScanningParams(const fanuc_grinding_scanning::ScanningService::Request &params);
  void setPublishParams(const fanuc_grinding_publish_meshfile::PublishMeshfileService::Request &params);

Q_SIGNALS:
  void guiChanged();
  void sendStatus(const QString status);
  void sendMsgBox(const QString title, const QString msg, const QString info_msg);
  void enablePanel(const bool);
  void enableScanningButton();
  void enablePanelAlignment();
  void enablePanelPathPlanning();
  void enableScanWidget();
  void sendCADDatas(const QString cad_path, const QString cad_marker_name);
  void sendScanDatas(const QString scan_path, const QString scan_marker_name);

protected Q_SLOTS:
  void connectToServices();
  void triggerSave();
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
  void publishCADMeshOrCloudFile();
  void publishScanMeshOrCloudFile();
  void scanning();
  void newStatusMessage(const std_msgs::String::ConstPtr &msg);

protected:
  const std::string package_name_;

  ros::NodeHandle nh_;
  ros::ServiceClient scanning_service_;
  fanuc_grinding_scanning::ScanningService srv_scanning_;
  ros::ServiceClient publish_meshfile_service_;
  fanuc_grinding_publish_meshfile::PublishMeshfileService srv_publish_meshfile_;
  ros::Subscriber status_sub_;

  // Import CAD
  QLineEdit* cad_meshname_;
  QPushButton* cad_meshname_browse_button_;
  QPushButton* import_cad_button_;

  // Give a marker name for CAD file
  QLineEdit* cad_marker_name_line_;

  // Container for all the scan import widgets
  QWidget* scan_choice_container_;
  QTabWidget *scan_choice_widget_;
  QWidget *start_scan_tab_;
  QWidget *import_scan_tab_;

  // Widget for scan
  // YAML part in order to parse joint values
  QLineEdit* traj_yaml_file_;
  QPushButton* traj_yaml_browse_button_;
  //Parameters for SLS-2
  QLineEdit* sls_2_server_name_;
  QLineEdit* sls_2_ip_address_;
  //YAML part in order to parse calibration sls2 matrix value
  QLineEdit* calibration_yaml_file_;
  QDoubleSpinBox* down_sampling_leaf_size_;
  QPushButton* calibration_yaml_browse_button_;

  //Widget for point cloud
  QLineEdit* scan_file_;
  QPushButton* scan_file_browse_button_;
  QPushButton* import_scan_;

  // Widget for scan file
  QLineEdit* scan_marker_name_line_;

  QPushButton* start_scan_;
};

} // End namespace

#endif // SCANNING_WIDGET_H
