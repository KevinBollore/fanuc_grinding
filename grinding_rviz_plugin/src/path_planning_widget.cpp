
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QFuture>
#include <QtConcurrentRun>
#include <QMessageBox>

#include "path_planning_widget.h"

grinding_rviz_plugin::PathPlanningWidget::PathPlanningWidget(QWidget* parent) : QWidget(parent)
{
  this->setObjectName("PathPlanningWidget_");
  covering_percentage_label_ = new QLabel;
  covering_percentage_label_->setText(QString::fromStdString("Covering percentage:"));
  covering_percentage_ = new QSpinBox;
  covering_percentage_->setSuffix(QString::fromStdString("%"));
  covering_percentage_->setValue(40);
  QHBoxLayout* covering_percentage_layout = new QHBoxLayout;
  covering_percentage_layout->addWidget(covering_percentage_label_);
  covering_percentage_layout->addWidget(covering_percentage_);

  extrication_frequency_label_ = new QLabel;
  extrication_frequency_label_->setText(QString::fromStdString("Extrication frequency:"));
  extrication_frequency_ = new QSpinBox;
  extrication_frequency_->setValue(5);
  QHBoxLayout* extrication_frequency_layout = new QHBoxLayout;
  extrication_frequency_layout->addWidget(extrication_frequency_label_);
  extrication_frequency_layout->addWidget(extrication_frequency_);

  extrication_coefficient_label_ = new QLabel;
  extrication_coefficient_label_->setText(QString::fromStdString("Extrication coefficient:"));
  extrication_coefficient_ = new QSpinBox;
  extrication_coefficient_->setValue(5);
  QHBoxLayout* extrication_coefficient_layout = new QHBoxLayout;
  extrication_coefficient_layout->addWidget(extrication_coefficient_label_);
  extrication_coefficient_layout->addWidget(extrication_coefficient_);

  grind_diameter_label_ = new QLabel;
  grind_diameter_label_->setText("Grinder diameter:");
  grind_diameter_ = new QDoubleSpinBox;
  grind_diameter_->setSuffix(" mm");
  grind_diameter_->setValue(30);
  grind_diameter_->setDecimals(1);
  QHBoxLayout* grind_diameter_layout = new QHBoxLayout;
  grind_diameter_layout->addWidget(grind_diameter_label_);
  grind_diameter_layout->addWidget(grind_diameter_);

  depth_label_ = new QLabel;
  depth_label_->setText("Depth of path:");
  depth_of_pass_ = new QDoubleSpinBox;
  depth_of_pass_->setSuffix(" mm");
  depth_of_pass_->setValue(1.0);
  depth_of_pass_->setDecimals(3);
  QHBoxLayout* select_depth_layout = new QHBoxLayout;
  select_depth_layout->addWidget(depth_label_);
  select_depth_layout->addWidget(depth_of_pass_);

  lean_angle_axis_label_ = new QLabel;
  lean_angle_axis_label_->setText(QString::fromStdString("Axis of rotation:"));
  lean_angle_axis_x_ = new QRadioButton;
  lean_angle_axis_x_->setText("x");
  lean_angle_axis_y_ = new QRadioButton;
  lean_angle_axis_y_->setText("y");
  lean_angle_axis_z_ = new QRadioButton;
  lean_angle_axis_z_->setText("z");
  QHBoxLayout* lean_angle_axis_layout = new QHBoxLayout;
  lean_angle_axis_layout->addWidget(lean_angle_axis_label_);
  lean_angle_axis_layout->addStretch(1);
  lean_angle_axis_layout->addWidget(lean_angle_axis_x_);
  lean_angle_axis_layout->addWidget(lean_angle_axis_y_);
  lean_angle_axis_layout->addWidget(lean_angle_axis_z_);
  lean_angle_axis_layout->addStretch(1);

  angle_value_label_ = new QLabel;
  angle_value_label_->setText(QString::fromStdString("Angle Value:"));
  angle_value_ = new QDoubleSpinBox;
  angle_value_->setSuffix(QString::fromStdString(" degrees"));
  angle_value_->setMinimum(-90);
  angle_value_->setMaximum(90);
  QHBoxLayout* angle_value_layout = new QHBoxLayout;
  angle_value_layout->addWidget(angle_value_label_);
  angle_value_layout->addWidget(angle_value_);

  compute_trajectory_ = new QPushButton;
  compute_trajectory_->setText("Compute trajectory");
  compute_trajectory_->setMinimumHeight(90);

  path_planning_simulation_ = new QPushButton;
  path_planning_simulation_->setText("Execute trajectory");
  path_planning_simulation_->setEnabled(false);
  path_planning_simulation_->setMinimumHeight(60);
  visualize_trajectory_ = new QPushButton;
  visualize_trajectory_->setText("Visualize trajectory");
  visualize_trajectory_->setEnabled(false);
  visualize_trajectory_->setMinimumHeight(60);
  QHBoxLayout* button_path_planning_layout = new QHBoxLayout;
  button_path_planning_layout->addWidget(visualize_trajectory_);
  button_path_planning_layout->addWidget(path_planning_simulation_);

  QVBoxLayout* path_planning_layout = new QVBoxLayout(this);
  path_planning_layout->addLayout(covering_percentage_layout);
  path_planning_layout->addLayout(extrication_frequency_layout);
  path_planning_layout->addLayout(extrication_coefficient_layout);
  path_planning_layout->addLayout(grind_diameter_layout);
  path_planning_layout->addLayout(select_depth_layout);
  path_planning_layout->addLayout(lean_angle_axis_layout);
  path_planning_layout->addLayout(angle_value_layout);
  path_planning_layout->addStretch(2);
  path_planning_layout->addWidget(compute_trajectory_);
  path_planning_layout->addStretch(1);
  path_planning_layout->addLayout(button_path_planning_layout);
  path_planning_layout->addStretch(8);

  //Connect handlers
  // At each modification of the widget, we call triggerSave
  connect(covering_percentage_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(extrication_frequency_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(extrication_coefficient_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(grind_diameter_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
  connect(depth_of_pass_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
  connect(lean_angle_axis_x_, SIGNAL(clicked()), this, SLOT(triggerSave()));
  connect(lean_angle_axis_y_, SIGNAL(clicked()), this, SLOT(triggerSave()));
  connect(lean_angle_axis_z_, SIGNAL(clicked()), this, SLOT(triggerSave()));
  connect(angle_value_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));

  // Enable or disable compute_trajectory_button_
  connect(this, SIGNAL(enableComputeTrajectoryButton(bool)), this, SLOT(enableComputeTrajectoryButtonHandler(bool)));

  // connect each buttons to different functions
  connect(compute_trajectory_, SIGNAL(released()), this, SLOT(ComputeTrajectoryButtonHandler()));
  connect(visualize_trajectory_, SIGNAL(released()), this, SLOT(VisualizeTrajectoryButtonHandler()));
  connect(path_planning_simulation_, SIGNAL(released()), this, SLOT(SimulateTrajectoryButtonHandler()));
  connect(this, SIGNAL(enableVizSimButton()), this, SLOT(enableVizSimButtonHandler()));

  // Subscriber to receive messages from the exterior
  status_sub_ = nh_.subscribe("path_planning_status", 1, &grinding_rviz_plugin::PathPlanningWidget::newStatusMessage, this);

  // Setup client
  path_planning_service_ = nh_.serviceClient<path_planning::PathPlanningService>("path_planning_service");
}

void grinding_rviz_plugin::PathPlanningWidget::newStatusMessage(const std_msgs::String::ConstPtr& msg)
{
  Q_EMIT sendStatus(QString::fromStdString(msg->data));
}

void grinding_rviz_plugin::PathPlanningWidget::enableComputeTrajectoryButtonHandler(bool state)
{
  compute_trajectory_->setEnabled(state);
}

path_planning::PathPlanningService::Request grinding_rviz_plugin::PathPlanningWidget::getPathPlanningParams()
{
  return path_planning_params_;
}

void grinding_rviz_plugin::PathPlanningWidget::setPathPlanningParams(path_planning::PathPlanningService::Request params)
{
  path_planning_params_.CoveringPercentage = params.CoveringPercentage;
  path_planning_params_.ExtricationFrequency = params.ExtricationFrequency;
  path_planning_params_.ExtricationCoefficient = params.ExtricationCoefficient;
  path_planning_params_.GrindDiameter = params.GrindDiameter;
  path_planning_params_.DepthOfPath = params.DepthOfPath;
  path_planning_params_.AngleX = params.AngleX;
  path_planning_params_.AngleY = params.AngleY;
  path_planning_params_.AngleZ = params.AngleZ;
  path_planning_params_.AngleValue = params.AngleValue;
  updateGUI();
}

void grinding_rviz_plugin::PathPlanningWidget::updateGUI()
{
  covering_percentage_->setValue(path_planning_params_.CoveringPercentage);
  extrication_frequency_->setValue(path_planning_params_.ExtricationFrequency);
  extrication_coefficient_->setValue(path_planning_params_.ExtricationCoefficient);
  grind_diameter_->setValue(path_planning_params_.GrindDiameter * 1000.0); // meters to millimeters
  depth_of_pass_->setValue(path_planning_params_.DepthOfPath * 1000.0); // meters to millimeters
  lean_angle_axis_x_->setChecked(path_planning_params_.AngleX);
  lean_angle_axis_y_->setChecked(path_planning_params_.AngleY);
  lean_angle_axis_z_->setChecked(path_planning_params_.AngleZ);
  angle_value_->setValue(path_planning_params_.AngleValue * 360.0 / M_PI); // radians to degrees
}

void grinding_rviz_plugin::PathPlanningWidget::updateInternalValues()
{
  path_planning_params_.CoveringPercentage = covering_percentage_->value();
  path_planning_params_.ExtricationFrequency = extrication_frequency_->value();
  path_planning_params_.ExtricationCoefficient = extrication_coefficient_->value();
  path_planning_params_.GrindDiameter = grind_diameter_->value() / 1000.0; // millimeters to meters
  path_planning_params_.DepthOfPath = depth_of_pass_->value() / 1000.0; // millimeters to meters
  path_planning_params_.AngleX = lean_angle_axis_x_->isChecked();
  path_planning_params_.AngleY = lean_angle_axis_y_->isChecked();
  path_planning_params_.AngleZ = lean_angle_axis_z_->isChecked();
  path_planning_params_.AngleValue = angle_value_->value() / 360.0 * M_PI; // degrees to radians
}

void grinding_rviz_plugin::PathPlanningWidget::setCADAndScanParams(const QString cad_filename,
                                                                const QString cad_marker_name,
                                                                const QString scan_filename,
                                                                const QString scan_marker_name)
{
  path_planning_params_.CADFileName = cad_filename.toStdString();
  path_planning_params_.CADMarkerName = cad_marker_name.toStdString();
  path_planning_params_.ScanFileName = scan_filename.toStdString();
  path_planning_params_.ScanMarkerName = scan_marker_name.toStdString();
}

void grinding_rviz_plugin::PathPlanningWidget::ComputeTrajectoryButtonHandler()
{
  // Fill service parameters with GUI values
  updateInternalValues();
  // get CAD and Scan params which are stored in grinding rviz plugin
  Q_EMIT getCADAndScanParams();

  // Fill in the request
  srv_path_planning_.request = getPathPlanningParams();
  srv_path_planning_.request.Compute = true;
  srv_path_planning_.request.Visualization = false;
  srv_path_planning_.request.Simulation = false;

  QFuture<void> future = QtConcurrent::run(this, &PathPlanningWidget::ComputeTrajectory);
}

void grinding_rviz_plugin::PathPlanningWidget::ComputeTrajectory()
{
  // Disable UI
  Q_EMIT enablePanel(false);
  Q_EMIT enableComputeTrajectoryButton(false);

  // Call client service
  path_planning_service_.call(srv_path_planning_);
  Q_EMIT sendStatus(QString::fromStdString(srv_path_planning_.response.ReturnMessage));

  if(srv_path_planning_.response.ReturnStatus == true)
  {
    // If visualization and simulation buttons are disabled, we put them to an enable state
    Q_EMIT enableVizSimButton();
    Q_EMIT enablePanelPostProcessor();
  }
  else
  {
    Q_EMIT sendMsgBox("Error importing mesh/point cloud file",
                      QString::fromStdString(srv_path_planning_.response.ReturnMessage), "");
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void grinding_rviz_plugin::PathPlanningWidget::enableVizSimButtonHandler()
{
  if(visualize_trajectory_->isEnabled() == false)
  {
    visualize_trajectory_->setEnabled(true);
  }
  if(path_planning_simulation_->isEnabled() == false)
  {
    path_planning_simulation_->setEnabled(true);
  }
}

void grinding_rviz_plugin::PathPlanningWidget::VisualizeTrajectoryButtonHandler()
{
  // If GUI has been changed, compute_trajectory_button_ is enabled.
  // So the pose is not up-to-date with GUI values
  // We inform the user that is an old pose
  if(compute_trajectory_->isEnabled() == true)
  {
    QMessageBox msgBox;
    msgBox.setText("Values have been modified");
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setInformativeText("If you continue, you will deal with an old trajectory");
    msgBox.setStandardButtons(QMessageBox::Abort | QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);

    if(msgBox.exec() == QMessageBox::Abort)
        return;
  }

  // get CAD and Scan params which are stored in grinding rviz plugin
  Q_EMIT getCADAndScanParams();

  // Fill in the request
  srv_path_planning_.request = getPathPlanningParams();
  srv_path_planning_.request.Compute = false;
  srv_path_planning_.request.Visualization = true;
  srv_path_planning_.request.Simulation = false;
  for (unsigned int i = 0; i < srv_path_planning_.response.RobotPosesOutput.size(); ++i)
  {
    // NB: RobotPoses and PointColorViz have same size
    srv_path_planning_.request.RobotPosesInput.push_back(srv_path_planning_.response.RobotPosesOutput[i]);
    srv_path_planning_.request.PointColorVizInput.push_back(srv_path_planning_.response.PointColorVizOutput[i]);
  }
  for (unsigned int j = 0; j < srv_path_planning_.response.IndexVectorOutput.size(); ++j)
  {
    srv_path_planning_.request.IndexVectorInput.push_back(srv_path_planning_.response.IndexVectorOutput[j]);
  }
  //srv_.request.*request* = *value*;
  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &PathPlanningWidget::VisualizeTrajectory);

}

void grinding_rviz_plugin::PathPlanningWidget::VisualizeTrajectory()
{
  // Disable UI
  Q_EMIT enablePanel(false);
  Q_EMIT sendStatus("Visualization of trajectories...");
  // Call client service
  path_planning_service_.call(srv_path_planning_);
  // Display return message in Qt panel
  Q_EMIT sendStatus(QString::fromStdString(srv_path_planning_.response.ReturnMessage));

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void grinding_rviz_plugin::PathPlanningWidget::SimulateTrajectoryButtonHandler()
{
  // If GUI has been changed, compute_trajectory_button_ is enable.
  // So, the pose is not up-to-date with GUI values
  // We inform the user that is an old pose
  if(compute_trajectory_->isEnabled() == true)
  {
    QMessageBox msgBox;
    msgBox.setText("Values have been modified");
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setInformativeText("If you continue, you will deal with an old trajectory");
    msgBox.setStandardButtons(QMessageBox::Abort | QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);

    if(msgBox.exec() == QMessageBox::Abort)
      return;
  }

  // get CAD and Scan params which are stored in grinding rviz plugin
  Q_EMIT getCADAndScanParams();

  // Fill in the request
  srv_path_planning_.request = getPathPlanningParams();
  srv_path_planning_.request.Compute = false;
  srv_path_planning_.request.Visualization = false;
  srv_path_planning_.request.Simulation = true;
  for (unsigned int i = 0; i < srv_path_planning_.response.RobotPosesOutput.size(); ++i)
  {
    // NB: RobotPoses and PointColorViz have same size
    srv_path_planning_.request.RobotPosesInput.push_back(srv_path_planning_.response.RobotPosesOutput[i]);
    srv_path_planning_.request.PointColorVizInput.push_back(srv_path_planning_.response.PointColorVizOutput[i]);
  }
  for (unsigned int j = 0; j < srv_path_planning_.response.IndexVectorOutput.size(); ++j)
  {
    srv_path_planning_.request.IndexVectorInput.push_back(srv_path_planning_.response.IndexVectorOutput[j]);
  }
  //srv_.request.*request* = *value*;
  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &PathPlanningWidget::SimulateTrajectory);
}

void grinding_rviz_plugin::PathPlanningWidget::SimulateTrajectory()
{
  // Disable UI
  Q_EMIT enablePanel(false);
  Q_EMIT sendStatus("Simulation of trajectories...");
  // Call client service
  path_planning_service_.call(srv_path_planning_);
  // Display return message in Qt panel
  Q_EMIT sendStatus(QString::fromStdString(srv_path_planning_.response.ReturnMessage));

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void grinding_rviz_plugin::PathPlanningWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT sendStatus("Connecting to service");
  while (ros::ok())
  {
    if (path_planning_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel connected to the service " << path_planning_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + path_planning_service_.getService()));
      break;
    }
    else
    {
      ROS_ERROR_STREAM("RViz panel could not connect to ROS service:\n\t" << path_planning_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + path_planning_service_.getService()));
      sleep(1);
    }
  }

  ROS_WARN_STREAM("Service connection have been made");
  Q_EMIT sendStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

void grinding_rviz_plugin::PathPlanningWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
  updateGUI();
  Q_EMIT enableComputeTrajectoryButton(true);
}

// Save all configuration data from this panel to the given Config object
void grinding_rviz_plugin::PathPlanningWidget::save(rviz::Config config)
{
  // Save offset value into the config file
  config.mapSetValue(this->objectName() + "covering_percentage", covering_percentage_->value());
  config.mapSetValue(this->objectName() + "extrication_frequency", extrication_frequency_->value());
  config.mapSetValue(this->objectName() + "extrication_coefficient", extrication_coefficient_->value());
  config.mapSetValue(this->objectName() + "grind_diameter", grind_diameter_->value());
  config.mapSetValue(this->objectName() + "depth_of_path", depth_of_pass_->value());
  config.mapSetValue(this->objectName() + "radio_x", lean_angle_axis_x_->isChecked());
  config.mapSetValue(this->objectName() + "radio_y", lean_angle_axis_y_->isChecked());
  config.mapSetValue(this->objectName() + "radio_z", lean_angle_axis_z_->isChecked());
  config.mapSetValue(this->objectName() + "angle_value", angle_value_->value());
}

// Load all configuration data for this panel from the given Config object.
void grinding_rviz_plugin::PathPlanningWidget::load(const rviz::Config& config)
{
  int tmp_int;
  bool tmp_bool;
  float tmp_float;
  QString tmp_string;
  // Load offset value from config file (if it exists)
  if (config.mapGetInt(this->objectName() + "covering_percentage", &tmp_int))
    covering_percentage_->setValue(tmp_int);
  else
    covering_percentage_->setValue(40);

  if (config.mapGetInt(this->objectName() + "extrication_frequency", &tmp_int))
    extrication_frequency_->setValue(tmp_int);
  else
    extrication_coefficient_->setValue(3);

  if (config.mapGetInt(this->objectName() + "extrication_coefficient", &tmp_int))
    extrication_coefficient_->setValue(tmp_int);
  else
    extrication_coefficient_->setValue(3);

  if (config.mapGetInt(this->objectName() + "grind_diameter", &tmp_int))
    grind_diameter_->setValue(tmp_int);
  else
    grind_diameter_->setValue(120);

  if (config.mapGetInt(this->objectName() + "depth_of_path", &tmp_int))
    depth_of_pass_->setValue(tmp_int);
  else
    depth_of_pass_->setValue(1.0);

  if (config.mapGetBool(this->objectName() + "radio_x", &tmp_bool))
    lean_angle_axis_x_->setChecked(tmp_bool);
  if (config.mapGetBool(this->objectName() + "radio_y", &tmp_bool))
    lean_angle_axis_y_->setChecked(tmp_bool);
  if (config.mapGetBool(this->objectName() + "radio_z", &tmp_bool))
    lean_angle_axis_z_->setChecked(tmp_bool);
  if (!lean_angle_axis_x_->isChecked() && !lean_angle_axis_y_->isChecked() && !lean_angle_axis_z_->isChecked())
  {
    lean_angle_axis_z_->setChecked(true);
  }

  if (config.mapGetFloat(this->objectName() + "angle_value", &tmp_float))
    angle_value_->setValue(tmp_float);
  else
    angle_value_->setValue(10);
}
