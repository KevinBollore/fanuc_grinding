#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QFuture>
#include <QtConcurrentRun>

#include "comparison_widget.h"

fanuc_grinding_rviz_plugin::ComparisonWidget::ComparisonWidget(QWidget* parent) : QWidget(parent)
{
  setObjectName("ComparisonWidget_");
  comparison_button_ = new QPushButton;
  comparison_button_->setText("Start Comparison");

  comparison_layout_ = new QVBoxLayout(this);
  comparison_layout_->addWidget(comparison_button_);

  // Connect handlers
  connect(comparison_button_, SIGNAL(released()), this, SLOT(comparisonButtonHandler()));

  //Setup client
  comparison_service_ = comparison_node_.serviceClient<fanuc_grinding_comparison::ComparisonService>("comparison_service");

  QFuture<void> future = QtConcurrent::run(this, &fanuc_grinding_rviz_plugin::ComparisonWidget::connectToServices);
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::triggerSave()
{
  Q_EMIT guiChanged();
  updateInternalValues();
  updateGUI();
}

fanuc_grinding_comparison::ComparisonService::Request fanuc_grinding_rviz_plugin::ComparisonWidget::getComparisonParams()
{
  return comparison_params_;
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::setComparisonParams(fanuc_grinding_comparison::ComparisonService::Request params)
{
  updateGUI();
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::updateGUI()
{
  // Not implemented yet
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::updateInternalValues()
{
  // Not implemented yet
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::setCADAndScanParams(const QString cad_filename,
                                                                const QString cad_marker_name,
                                                                const QString scan_filename,
                                                                const QString scan_marker_name)
{
  comparison_params_.CADFileName = cad_filename.toStdString();
  comparison_params_.CADMarkerName = cad_marker_name.toStdString();
  comparison_params_.ScanFileName = scan_filename.toStdString();
  comparison_params_.ScanMarkerName = scan_marker_name.toStdString();
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::comparisonButtonHandler()
{
  // get CAD and Scan params which are stored in grinding rviz plugin
  Q_EMIT getCADAndScanParams();
  // Fill in the request
  srv_comparison_.request = getComparisonParams();
  //srv_.request.*request* = *value*;
  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &ComparisonWidget::comparison);
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::comparison()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service
  comparison_service_.call(srv_comparison_);
  Q_EMIT sendStatus(QString::fromStdString(srv_comparison_.response.ReturnMessage));

  if(srv_comparison_.response.ReturnStatus == true)
    Q_EMIT enablePanelPathPlanning();
  else
  {
    Q_EMIT sendMsgBox("Error comparing meshes",
                      QString::fromStdString(srv_comparison_.response.ReturnMessage), "");
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void fanuc_grinding_rviz_plugin::ComparisonWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT sendStatus("Connecting to service");
  while (ros::ok())
  {
    if (comparison_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(objectName().toStdString() + " RViz panel connected to the service " << comparison_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + comparison_service_.getService()));
      break;
    }
    else
    {
      ROS_WARN_STREAM(objectName().toStdString() + " RViz panel could not connect to ROS service:\n\t" << comparison_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + comparison_service_.getService()));
      sleep(1);
    }
  }

  ROS_INFO_STREAM(objectName().toStdString() + " service connections have been made");
  Q_EMIT sendStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

// Save all configuration data from this panel to the given Config object
void fanuc_grinding_rviz_plugin::ComparisonWidget::save(rviz::Config config)
{
  // NOT IMPLEMENTED YET
}

// Load all configuration data for this panel from the given Config object.
void fanuc_grinding_rviz_plugin::ComparisonWidget::load(const rviz::Config& config)
{
  // NOT IMPLEMENTED YET
}
