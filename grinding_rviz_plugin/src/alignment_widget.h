#ifndef ALIGNMENT_WIDGET_H
#define ALIGNMENT_WIDGET_H

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
class QSpinBox;
class QDoubleSpinBox;

#include <alignment/AlignmentService.h>

namespace grinding_rviz_plugin
{
class AlignmentWidget : public QWidget
{

  Q_OBJECT
public:
  AlignmentWidget(QWidget* parent =  NULL);

  alignment::AlignmentService::Request getAlignmentParams();
  void setAlignmentParams(alignment::AlignmentService::Request params);

  void connectToServices();

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();
  void enablePanel(bool);
  void sendStatus(QString status);
  void sendMsgBox(QString title, QString msg, QString info_msg);
  void enablePanelComparison();
  void getCADAndScanParams();

public Q_SLOTS:
  virtual void Alignment();

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void AlignmentButtonHandler();
  void enablePanelComparisonHandler();
  void setCADAndScanParams(const QString cad_filename,
                           const QString cad_marker_name,
                           const QString scan_filename,
                           const QString scan_marker_name);

protected:
  //ROS
  ros::NodeHandle nh_;
  ros::ServiceClient alignment_service_;
  alignment::AlignmentService srv_alignment_;

  alignment::AlignmentService::Request alignment_params_;

  QLabel *uniform_sampling_label_;
  QLabel *number_samples_label_;
  QSpinBox *number_samples_;

  QLabel *voxel_grid_label_;
  QLabel *leaf_size_label_;
  QDoubleSpinBox *leaf_size_;

  QLabel *icp_label_;
  QLabel *iteration_number_label_;
  QSpinBox *iteration_number_;
  QLabel *max_iteration_number_label_;
  QSpinBox *max_iteration_number_;
  QLabel *max_corr_dist_label_;
  QDoubleSpinBox *max_corr_dist_;

  QPushButton *alignment_button_;
};

} // End namespace


#endif // ALIGNMENT_WIDGET_H
