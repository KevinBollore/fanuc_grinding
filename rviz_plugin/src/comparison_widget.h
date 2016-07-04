#ifndef COMPARISON_WIDGET_H
#define COMPARISON_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <fanuc_grinding_comparison/ComparisonService.h>
#endif

class QLabel;
class QVBoxLayout;
class QPushButton;

namespace fanuc_grinding_rviz_plugin
{
class ComparisonWidget : public QWidget
{
  Q_OBJECT
public:
  ComparisonWidget(QWidget* parent =  NULL);
  void load(const rviz::Config& config);
  void save(rviz::Config config);
  void setComparisonParams(const fanuc_grinding_comparison::ComparisonService::Request &params);

Q_SIGNALS:
  void guiChanged();
  void sendStatus(const QString status);
  void sendMsgBox(const QString title, const QString msg, const QString info_msg);
  void enablePanel(bool);
  void enablePanelPathPlanning();
  void getCADAndScanParams();

protected Q_SLOTS:
  void connectToServices();
  void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void comparisonButtonHandler();
  void comparison();
  void enablePanelPathPlanningHandler();
  void setCADAndScanParams(const QString cad_filename,
                           const QString scan_filename);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient comparison_service_;
  fanuc_grinding_comparison::ComparisonService srv_comparison_;

  QPushButton* comparison_button_;
};

} // End namespace

#endif // COMPARISON_WIDGET_H
