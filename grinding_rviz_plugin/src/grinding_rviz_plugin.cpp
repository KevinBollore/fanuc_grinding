#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QMessageBox>
#include <QFuture>
#include <QtConcurrentRun>
#include <QWidget>
#include <QTabWidget>
#include <QDebug>
#include <QLabel>
#include <QRadioButton>
#include <QLineEdit>
#include <QFileDialog>

#include <string>
#include <iostream>

#include "grinding_rviz_plugin.h"

#include <scanning/ScanningService.h>
#include <alignment/AlignmentService.h>
#include <comparison/ComparisonService.h>
#include <path_planning/PathPlanningService.h>
#include <publish_meshfile/PublishMeshfileService.h>
#include <post_processor/PostProcessorService.h>

namespace grinding_rviz_plugin
{
GrindingRvizPlugin::GrindingRvizPlugin(QWidget* parent) :
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
  tab_widget_->setTabEnabled(1, false);
  tab_widget_->setTabEnabled(2, false);
  tab_widget_->setTabEnabled(3, false);
  tab_widget_->setTabEnabled(4, false);

  // Bottom status layout
  QVBoxLayout *status_layout_ = new QVBoxLayout;
  status_layout_->addWidget(new QLabel("Status:"));

  // Global Layout
  QVBoxLayout *global_layout_ = new QVBoxLayout;
  global_layout_->addWidget(tab_widget_);
  global_layout_->addLayout(status_layout_);
  status_label_ = new QLabel;
  global_layout_->addWidget(status_label_);
  setLayout(global_layout_);

  // Connect handlers
  // SCANNING
  // Will display a status in general status label ( from scanning widget )
  connect(scanning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(scanning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)),
                      this, SLOT(displayMsgBoxHandler(QString, QString, QString)));

  // Call configChanged at each time that scanning_widget_ is modified
  connect(scanning_widget_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  // Enable AlignementPanel when scanning_widget_ will send the SIGNAL
  connect(scanning_widget_, SIGNAL(enablePanelAlignment()), this, SLOT(enablePanelAlignmentHandler()));
  // Enable general panel when scanning_widget_ send the SIGNAL
  connect(scanning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));

  // Will send information about cad and scan in the other widgets
  connect(scanning_widget_, SIGNAL(sendCADDatas(QString, QString)),
                      this, SLOT(setCADDatas(QString, QString)));
  connect(scanning_widget_, SIGNAL(sendScanDatas(QString, QString)),
                      this, SLOT(setScanDatas(QString, QString)));
  // For the demonstrator, we will skip alignment and comparison parts for the moment
  connect(scanning_widget_, SIGNAL(enablePanelPathPlanning()), this, SLOT(enablePanelPathPlanningHandler()));

  //ALIGNMENT
  // Will display a status in general status label ( from alignment widget )
  connect(alignment_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(alignment_widget_, SIGNAL(sendMsgBox(QString, QString , QString)),
                       this, SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that alignment_widget_ is modified
  connect(alignment_widget_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  // Enable compoarison_panel when alignment_widget_ will send the SIGNAL
  connect(alignment_widget_,SIGNAL(enablePanelComparison()), this, SLOT(enablePanelComparisonHandler()));
  // Enable general panel when alignment_widget_ send the SIGNAL
  connect(alignment_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from alignment widget in order to get CAD and scan params
  connect(alignment_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to alignment widget in order to give CAD and scan params
  connect(this             , SIGNAL(sendCADAndScanDatas(const QString, const QString, const QString, const QString)),
          alignment_widget_, SLOT(setCADAndScanParams(const QString, const QString, const QString, const QString)));

  //COMPARISON
  // Will display a status in general status label ( from comparison widget )
  connect(comparison_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  // Call configChanged at each time that comparison_widget_ is modified
  connect(comparison_widget_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  // Enable path_planning_widget when comparison_widget_ will send the SIGNAL
  connect(comparison_widget_,SIGNAL(enablePanelPathPlanning()), this, SLOT(enablePanelPathPlanningHandler()));
  // Enable general panel when comparison_widget_ send the SIGNAL
  connect(comparison_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from comparison widget in order to get CAD and scan params
  connect(comparison_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to comparison widget in order to give CAD and scan params
  connect(this             , SIGNAL(sendCADAndScanDatas(const QString, const QString, const QString, const QString)),
          comparison_widget_, SLOT(setCADAndScanParams(const QString, const QString, const QString, const QString)));

  //PATH PLANNING
  // Will display a status in general status label ( from path_planning widget )
  connect(path_planning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(path_planning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)),
                           this, SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that path_planning_widget is modified
  connect(path_planning_widget_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  // Enable path_planning_widget when comparison_widget_ will send the SIGNAL
  connect(path_planning_widget_, SIGNAL(enablePanelPostProcessor()), this, SLOT(enablePanelPostProcessorHandler()));
  // Enable general panel when path_planning send the SIGNAL
  connect(path_planning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from comparison widget in order to get CAD and scan params
  connect(path_planning_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to comparison widget in order to give CAD and scan params
  connect(this             , SIGNAL(sendCADAndScanDatas(const QString, const QString, const QString, const QString)),
          path_planning_widget_, SLOT(setCADAndScanParams(const QString, const QString, const QString, const QString)));

  //POST_PROCESSOR
  // Will display a status in general status label ( from post_processor widget )
  connect(post_processor_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  // Call configChanged at each time that post_processor_widget is modified
  connect(post_processor_widget_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  // Enable general panel when post_processor send the SIGNAL
  connect(post_processor_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Receive a signal from post processor widget in order to send robot poses data
  connect(post_processor_widget_, SIGNAL(getRobotPosesData()), this, SLOT(setRobotPosesData()));

  connect(this, SIGNAL(displayStatus(const QString)), this, SLOT(displayStatusHandler(const QString)));

  // Check connection of client
  QFuture<void> future = QtConcurrent::run(this, &GrindingRvizPlugin::connectToServices);
}

GrindingRvizPlugin::~GrindingRvizPlugin()
{}

void GrindingRvizPlugin::enablePanelAlignmentHandler()
{
  tab_widget_->setTabEnabled(1, true);
}

void GrindingRvizPlugin::enablePanelComparisonHandler()
{
  tab_widget_->setTabEnabled(2, true);
}

void GrindingRvizPlugin::enablePanelPathPlanningHandler()
{
  tab_widget_->setTabEnabled(3, true);
}

void GrindingRvizPlugin::enablePanelPostProcessorHandler()
{
  tab_widget_->setTabEnabled(4, true);
}

void GrindingRvizPlugin::enablePanelHandler(bool status)
{
  this->setEnabled(status);
}

void GrindingRvizPlugin::displayStatusHandler(const QString message)
{
  status_label_->setText(message);
}

void GrindingRvizPlugin::displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg)
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

void GrindingRvizPlugin::triggerSave()
{
  Q_EMIT configChanged();
}

void GrindingRvizPlugin::setCADDatas(const QString cad_filename, const QString cad_marker_name)
{
  cad_filename_ = cad_filename;
  cad_marker_name_ = cad_marker_name;
}

void GrindingRvizPlugin::setScanDatas(const QString scan_filename, const QString scan_marker_name)
{
  scan_filename_ = scan_filename;
  scan_marker_name_ = scan_marker_name;
}

void GrindingRvizPlugin::setRobotPosesData()
{
  post_processor_widget_->setRobotPoses(path_planning_widget_->getRobotPoses());
  post_processor_widget_->setPointColorViz(path_planning_widget_->getPointColorViz());
  post_processor_widget_->setIndexVector(path_planning_widget_->getIndexVector());
}

void GrindingRvizPlugin::sendCADAndScanDatasSlot()
{
  Q_EMIT sendCADAndScanDatas(cad_filename_, cad_marker_name_, scan_filename_, scan_marker_name_);
}

void GrindingRvizPlugin::connectToServices()
{
  scanning_widget_->connectToServices();
  alignment_widget_->connectToServices();
  comparison_widget_->connectToServices();
  path_planning_widget_->connectToServices();
  post_processor_widget_->connectToServices();
}

// Save all configuration data from this panel to the given Config object
void GrindingRvizPlugin::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  scanning_widget_->save(config);
  alignment_widget_->save(config);
  comparison_widget_->save(config);
  path_planning_widget_->save(config);
  post_processor_widget_->save(config);
}

// Load all configuration data for this panel from the given Config object.
void GrindingRvizPlugin::load(const rviz::Config& config)
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
PLUGINLIB_EXPORT_CLASS(grinding_rviz_plugin::GrindingRvizPlugin, rviz::Panel)
