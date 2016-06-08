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

public Q_SLOTS:
  virtual void Alignment();

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void AlignmentButtonHandler();
  void enablePanelComparisonHandler();

protected:
  //ROS
  ros::NodeHandle nh_;
  ros::ServiceClient alignment_service_;
  alignment::AlignmentService srv_alignment_;

  alignment::AlignmentService::Request alignment_params_;

  QPushButton *alignment_button_;

};

} // End namespace


#endif // ALIGNMENT_WIDGET_H
