#ifndef FANUC_GRINDING_RVIZ_PLUGIN_H
#define FANUC_GRINDING_RVIZ_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include "scanning_widget.h"
#include "alignment_widget.h"
#include "comparison_widget.h"
#include "path_planning_widget.h"
#include "post_processor_widget.h"
#endif

class QTabWidget;
class QLabel;
class QVBoxLayout;

namespace fanuc_grinding_rviz_plugin
{
class FanucGrindingRvizPlugin : public rviz::Panel
{
  Q_OBJECT
public:
  FanucGrindingRvizPlugin(QWidget* parent = 0);
  virtual ~FanucGrindingRvizPlugin();

  Q_SIGNALS:
  void enableWidget(bool);
  void displayStatus(const QString);
  void sendCADAndScanDatas(const QString, const QString, const QString, const QString);

protected Q_SLOTS:
  virtual void
  triggerSave();

  void displayStatusHandler(const QString message);
  void displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg);

  void enablePanelHandler(bool);
  void enablePanelAlignmentHandler();
  void enablePanelComparisonHandler();
  void enablePanelPathPlanningHandler();
  void enablePanelPostProcessorHandler();
  void setCADDatas(const QString cad_path, const QString cad_marker_name);
  void setScanDatas(const QString scan_path, const QString scan_marker_name);
  void sendCADAndScanDatasSlot();
  void setRobotTrajectoryData();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected:

  // Qt Interface
  // Tabs
  QTabWidget *tab_widget_;
  ScanningWidget *scanning_widget_;
  AlignmentWidget *alignment_widget_;
  ComparisonWidget *comparison_widget_;
  PathPlanningWidget *path_planning_widget_;
  PostProcessorWidget *post_processor_widget_;

  // Status Layout
  QVBoxLayout *status_layout_;
  QLabel *status_label_;

  // Global Layout
  QVBoxLayout *global_layout_;

private:
  QString cad_filename_;
  QString cad_marker_name_;
  QString scan_filename_;
  QString scan_marker_name_;
};

}  // end namespace

#endif // FANUC_GRINDING_RVIZ_PLUGIN_H
