#ifndef COMPARISON_WIDGET_H
#define COMPARISON_WIDGET_H

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

#include <comparison/ComparisonService.h>

namespace grinding_rviz_plugin
{
class ComparisonWidget : public QWidget
{

  Q_OBJECT
public:
  ComparisonWidget(QWidget* parent =  NULL);

  comparison::ComparisonService::Request getComparisonParams();
  void setComparisonParams(comparison::ComparisonService::Request params);

  void connectToServices();

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();
  void enablePanelPathPlanning();
  void sendStatus(QString status);
  void enablePanel(bool);

public Q_SLOTS:
  virtual void Comparison();

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void ComparisonButtonHandler();
  void enablePanelPathPlanningHandler();

protected:
  //ROS
  ros::NodeHandle comparison_node_;
  ros::ServiceClient comparison_service_;
  comparison::ComparisonService srv_comparison_;

  comparison::ComparisonService::Request comparison_params_;

  QPushButton *comparison_button_;
  QVBoxLayout *comparison_layout_;
};

} // End namespace


#endif // COMPARISON_WIDGET_H
