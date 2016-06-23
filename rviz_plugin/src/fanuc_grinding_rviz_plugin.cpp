#include <QVBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QFuture>
#include <QtConcurrentRun>
#include <QWidget>
#include <QTabWidget>

#include "fanuc_grinding_rviz_plugin.h"

namespace fanuc_grinding_rviz_plugin
{
FanucGrindingRvizPlugin::FanucGrindingRvizPlugin(QWidget* parent) :
    rviz::Panel(parent)
{
  // Create Tabs
  tab_widget_ = new QTabWidget();

  scanning_widget_ = new ScanningWidget();
  alignment_widget_ = new AlignmentWidget();
  comparison_widget_ = new ComparisonWidget();
  path_planning_widget_ = new PathPlanningWidget();
  post_processor_widget_ = new PostProcessorWidget();

  tab_widget_->addTab(scanning_widget_, "Scanning");
  tab_widget_->addTab(alignment_widget_, "Alignment");
  tab_widget_->addTab(comparison_widget_, "Comparison");
  tab_widget_->addTab(path_planning_widget_, "Path planning");
  tab_widget_->addTab(post_processor_widget_, "Post processor");
  tab_widget_->setTabEnabled(0, true);
  tab_widget_->setTabEnabled(1, false);
  tab_widget_->setTabEnabled(2, false);
  tab_widget_->setTabEnabled(3, false);
  tab_widget_->setTabEnabled(4, false);

  // Bottom status layout
  QVBoxLayout* status_layout = new QVBoxLayout;
  status_layout->addWidget(new QLabel("Status:"));

  // Global Layout
  QVBoxLayout* global_layout = new QVBoxLayout;
  global_layout->addWidget(tab_widget_);
  global_layout->addLayout(status_layout);
  status_label_ = new QLabel;
  global_layout->addWidget(status_label_);
  setLayout(global_layout);

  // Connect handlers
  // SCANNING
  // Will display a status in general status label ( from scanning widget )
  connect(scanning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(scanning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));

  // Call configChanged at each time that scanning_widget_ is modified
  connect(scanning_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable AlignementPanel when scanning_widget_ will send the SIGNAL
  connect(scanning_widget_, SIGNAL(enablePanelAlignment()), this, SLOT(enablePanelAlignmentHandler()));
  // Enable general panel when scanning_widget_ send the SIGNAL
  connect(scanning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));

  // Will send information about cad and scan in the other widgets
  connect(scanning_widget_, SIGNAL(sendCADDatas(QString, QString)), this, SLOT(setCADDatas(QString, QString)));
  connect(scanning_widget_, SIGNAL(sendScanDatas(QString, QString)), this, SLOT(setScanDatas(QString, QString)));
  // For the demonstrator, we will skip alignment and comparison parts for the moment
  connect(scanning_widget_, SIGNAL(enablePanelPathPlanning()), this, SLOT(enablePanelPathPlanningHandler()));

  //ALIGNMENT
  // Will display a status in general status label ( from alignment widget )
  connect(alignment_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(alignment_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that alignment_widget_ is modified
  connect(alignment_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable compoarison_panel when alignment_widget_ will send the SIGNAL
  connect(alignment_widget_, SIGNAL(enablePanelComparison()), this, SLOT(enablePanelComparisonHandler()));
  // Enable general panel when alignment_widget_ send the SIGNAL
  connect(alignment_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from alignment widget in order to get CAD and scan params
  connect(alignment_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to alignment widget in order to give CAD and scan params
  connect(this, SIGNAL(sendCADAndScanDatas(const QString, const QString, const QString, const QString)),
          alignment_widget_, SLOT(setCADAndScanParams(const QString, const QString, const QString, const QString)));

  //COMPARISON
  // Will display a status in general status label ( from comparison widget )
  connect(comparison_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(comparison_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that comparison_widget_ is modified
  connect(comparison_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable path_planning_widget when comparison_widget_ will send the SIGNAL
  connect(comparison_widget_, SIGNAL(enablePanelPathPlanning()), this, SLOT(enablePanelPathPlanningHandler()));
  // Enable general panel when comparison_widget_ send the SIGNAL
  connect(comparison_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from comparison widget in order to get CAD and scan params
  connect(comparison_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to comparison widget in order to give CAD and scan params
  connect(this, SIGNAL(sendCADAndScanDatas(const QString, const QString, const QString, const QString)),
          comparison_widget_, SLOT(setCADAndScanParams(const QString, const QString, const QString, const QString)));

  //PATH PLANNING
  // Will display a status in general status label ( from path_planning widget )
  connect(path_planning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(path_planning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that path_planning_widget is modified
  connect(path_planning_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable path_planning_widget when comparison_widget_ will send the SIGNAL
  connect(path_planning_widget_, SIGNAL(enablePanelPostProcessor()), this, SLOT(enablePanelPostProcessorHandler()));
  // Enable general panel when path_planning send the SIGNAL
  connect(path_planning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from comparison widget in order to get CAD and scan params
  connect(path_planning_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to comparison widget in order to give CAD and scan params
  connect(this, SIGNAL(sendCADAndScanDatas(const QString, const QString, const QString, const QString)),
          path_planning_widget_, SLOT(setCADAndScanParams(const QString, const QString, const QString, const QString)));

  //POST_PROCESSOR
  // Will display a status in general status label ( from post_processor widget )
  connect(post_processor_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(post_processor_widget_, SIGNAL(sendMsgBox(QString, QString, QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that post_processor_widget is modified
  connect(post_processor_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable general panel when post_processor send the SIGNAL
  connect(post_processor_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Receive a signal from post processor widget in order to send robot poses data
  connect(post_processor_widget_, SIGNAL(getRobotTrajectoryData()), this, SLOT(setRobotTrajectoryData()));

  connect(this, SIGNAL(displayStatus(const QString)), this, SLOT(displayStatusHandler(const QString)));
}

FanucGrindingRvizPlugin::~FanucGrindingRvizPlugin()
{}

void FanucGrindingRvizPlugin::enablePanelAlignmentHandler()
{
  tab_widget_->setTabEnabled(1, true);
}

void FanucGrindingRvizPlugin::enablePanelComparisonHandler()
{
  tab_widget_->setTabEnabled(2, true);
}

void FanucGrindingRvizPlugin::enablePanelPathPlanningHandler()
{
  tab_widget_->setTabEnabled(3, true);
}

void FanucGrindingRvizPlugin::enablePanelPostProcessorHandler()
{
  tab_widget_->setTabEnabled(4, true);
}

void FanucGrindingRvizPlugin::enablePanelHandler(bool status)
{
  setEnabled(status);
}

void FanucGrindingRvizPlugin::displayStatusHandler(const QString message)
{
  status_label_->setText(message);
}

void FanucGrindingRvizPlugin::displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg)
{
  enablePanelHandler(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(msg);
  msg_box.setInformativeText(info_msg);
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  enablePanelHandler(true);
}

void FanucGrindingRvizPlugin::triggerSave()
{
  Q_EMIT configChanged();
}

void FanucGrindingRvizPlugin::setCADDatas(const QString cad_filename, const QString cad_marker_name)
{
  cad_filename_ = cad_filename;
  cad_marker_name_ = cad_marker_name;
}

void FanucGrindingRvizPlugin::setScanDatas(const QString scan_filename, const QString scan_marker_name)
{
  scan_filename_ = scan_filename;
  scan_marker_name_ = scan_marker_name;
}

void FanucGrindingRvizPlugin::setRobotTrajectoryData()
{
  post_processor_widget_->setRobotPoses(path_planning_widget_->getRobotPoses());
  post_processor_widget_->setPointColorViz(path_planning_widget_->getPointColorViz());
  post_processor_widget_->setIndexVector(path_planning_widget_->getIndexVector());
}

void FanucGrindingRvizPlugin::sendCADAndScanDatasSlot()
{
  Q_EMIT sendCADAndScanDatas(cad_filename_, cad_marker_name_, scan_filename_, scan_marker_name_);
}

// Save all configuration data from this panel to the given Config object
void FanucGrindingRvizPlugin::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  scanning_widget_->save(config);
  alignment_widget_->save(config);
  comparison_widget_->save(config);
  path_planning_widget_->save(config);
  post_processor_widget_->save(config);
}

// Load all configuration data for this panel from the given Config object.
void FanucGrindingRvizPlugin::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  scanning_widget_->load(config);
  alignment_widget_->load(config);
  comparison_widget_->load(config);
  path_planning_widget_->load(config);
  post_processor_widget_->load(config);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fanuc_grinding_rviz_plugin::FanucGrindingRvizPlugin, rviz::Panel)
