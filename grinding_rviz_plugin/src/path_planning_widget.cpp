
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
  CAD_label_ = new QLabel;
  CAD_label_->setText(QString::fromStdString("CAD File:"));
  CAD_line_ = new QLineEdit;
  //CAD_line_->setReadOnly(true);
  CAD_line_->setText(QString::fromStdString("/home/dell/catkin_workspace/src/bezier/bezier_application/meshes/plane/planeBIN.ply"));
  QHBoxLayout* CAD_layout = new QHBoxLayout;
  CAD_layout->addWidget(CAD_label_);
  CAD_layout->addWidget(CAD_line_);

  defect_mesh_label_ = new QLabel;
  defect_mesh_label_->setText(QString::fromStdString("Defect:"));
  defect_mesh_line_ = new QLineEdit;
  //defect_mesh_line_->setReadOnly(true);
  defect_mesh_line_->setText(QString::fromStdString("/home/dell/catkin_workspace/src/bezier/bezier_application/meshes/plane/planeBIN_defect.ply"));
  QHBoxLayout* defect_mesh_layout = new QHBoxLayout;
  defect_mesh_layout->addWidget(defect_mesh_label_);
  defect_mesh_layout->addWidget(defect_mesh_line_);

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
  grind_diameter_label_->setText(QString::fromStdString("Grind diameter:"));
  grind_diameter_ = new QSpinBox;
  grind_diameter_->setSuffix(QString::fromStdString("mm"));
  grind_diameter_->setValue(30);
  QHBoxLayout* grind_diameter_layout = new QHBoxLayout;
  grind_diameter_layout->addWidget(grind_diameter_label_);
  grind_diameter_layout->addWidget(grind_diameter_);

  depth_label_ = new QLabel;
  depth_label_->setText(QString::fromStdString("Depth of path: "));
  depth_spin_box_ = new QSpinBox;
  depth_spin_box_->setSuffix(QString::fromStdString("mm"));
  depth_spin_box_->setValue(15);
  QHBoxLayout* select_depth_layout = new QHBoxLayout;
  select_depth_layout->addWidget(depth_label_);
  select_depth_layout->addWidget(depth_spin_box_);

  lean_angle_axis_label_ = new QLabel;
  lean_angle_axis_label_->setText(QString::fromStdString("Axis of rotation:"));
  lean_angle_axis_x_radio_ = new QRadioButton;
  lean_angle_axis_x_radio_->setText("x");
  lean_angle_axis_y_radio_ = new QRadioButton;
  lean_angle_axis_y_radio_->setText("y");
  lean_angle_axis_z_radio_ = new QRadioButton;
  lean_angle_axis_z_radio_->setText("z");
  QHBoxLayout* lean_angle_axis_layout = new QHBoxLayout;
  lean_angle_axis_layout->addWidget(lean_angle_axis_label_);
  lean_angle_axis_layout->addWidget(lean_angle_axis_x_radio_);
  lean_angle_axis_layout->addWidget(lean_angle_axis_y_radio_);
  lean_angle_axis_layout->addWidget(lean_angle_axis_z_radio_);

  angle_value_label_ = new QLabel;
  angle_value_label_->setText(QString::fromStdString("Angle Value: "));
  angle_value_spin_box_ = new QDoubleSpinBox;
  angle_value_spin_box_->setSuffix(QString::fromStdString(" radians"));
  angle_value_spin_box_->setMinimum(-6.2831);
  angle_value_spin_box_->setMaximum(6.2831);
  QHBoxLayout* angle_value_layout = new QHBoxLayout;
  angle_value_layout->addWidget(angle_value_label_);
  angle_value_layout->addWidget(angle_value_spin_box_);

  compute_trajectory_button_ = new QPushButton;
  compute_trajectory_button_->setText("Compute trajectory");

  path_planning_simulation_button_ = new QPushButton;
  path_planning_simulation_button_->setText("Simulate trajectory");
  path_planning_simulation_button_->setEnabled(false);
  visualize_trajectory_button_ = new QPushButton;
  visualize_trajectory_button_->setText("Visualize trajectory");
  visualize_trajectory_button_->setEnabled(false);
  QHBoxLayout* button_path_planning_layout = new QHBoxLayout;
  button_path_planning_layout->addWidget(visualize_trajectory_button_);
  button_path_planning_layout->addWidget(path_planning_simulation_button_);

  generate_trajectory_button_ = new QPushButton;
  generate_trajectory_button_->setText("Generate trajectory");
  generate_trajectory_button_->setEnabled(false);

  QVBoxLayout* path_planning_layout = new QVBoxLayout(this);
  path_planning_layout->addLayout(CAD_layout);
  path_planning_layout->addLayout(defect_mesh_layout);
  path_planning_layout->addLayout(covering_percentage_layout);
  path_planning_layout->addLayout(extrication_frequency_layout);
  path_planning_layout->addLayout(extrication_coefficient_layout);
  path_planning_layout->addLayout(grind_diameter_layout);
  path_planning_layout->addLayout(select_depth_layout);
  path_planning_layout->addLayout(lean_angle_axis_layout);
  path_planning_layout->addLayout(angle_value_layout);
  path_planning_layout->addWidget(compute_trajectory_button_);
  path_planning_layout->addLayout(button_path_planning_layout);
  path_planning_layout->addWidget(generate_trajectory_button_);

  //Connect handlers
  // At each modification of the widget, we call triggerSave
  connect(CAD_line_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(defect_mesh_line_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(covering_percentage_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(extrication_frequency_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(extrication_coefficient_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(grind_diameter_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(depth_spin_box_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(lean_angle_axis_x_radio_, SIGNAL(clicked()), this, SLOT(triggerSave()));
  connect(lean_angle_axis_y_radio_, SIGNAL(clicked()), this, SLOT(triggerSave()));
  connect(lean_angle_axis_z_radio_, SIGNAL(clicked()), this, SLOT(triggerSave()));
  connect(angle_value_spin_box_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));

  // Enable or disable compute_trajectory_button_
  connect(this, SIGNAL(enableComputeTrajectoryButton(bool)), this, SLOT(enableComputeTrajectoryButtonHandler(bool)));

  // connect each buttons to different functions
  connect(compute_trajectory_button_, SIGNAL(released()), this, SLOT(ComputeTrajectoryButtonHandler()));
  connect(visualize_trajectory_button_, SIGNAL(released()), this, SLOT(VisualizeTrajectoryButtonHandler()));
  connect(path_planning_simulation_button_, SIGNAL(released()), this, SLOT(SimulateTrajectoryButtonHandler()));
  connect(generate_trajectory_button_, SIGNAL(released()), this, SLOT(generateTrajectoryButtonHandler()));
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
  compute_trajectory_button_->setEnabled(state);
}

path_planning::PathPlanningService::Request grinding_rviz_plugin::PathPlanningWidget::getPathPlanningParams()
{
  return path_planning_params_;
}

void grinding_rviz_plugin::PathPlanningWidget::setPathPlanningParams(path_planning::PathPlanningService::Request params)
{
  path_planning_params_.CADFile = params.CADFile;
  path_planning_params_.DefectFile = params.DefectFile;
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
  CAD_line_->setText(QString::fromStdString(path_planning_params_.CADFile));
  defect_mesh_line_->setText(QString::fromStdString(path_planning_params_.DefectFile));
  covering_percentage_->setValue(path_planning_params_.CoveringPercentage);
  extrication_frequency_->setValue(path_planning_params_.ExtricationFrequency);
  extrication_coefficient_->setValue(path_planning_params_.ExtricationCoefficient);
  grind_diameter_->setValue(path_planning_params_.GrindDiameter);
  depth_spin_box_->setValue(path_planning_params_.DepthOfPath);
  lean_angle_axis_x_radio_->setChecked(path_planning_params_.AngleX);
  lean_angle_axis_y_radio_->setChecked(path_planning_params_.AngleY);
  lean_angle_axis_z_radio_->setChecked(path_planning_params_.AngleZ);
  angle_value_spin_box_->setValue(path_planning_params_.AngleValue);
}

void grinding_rviz_plugin::PathPlanningWidget::updateInternalValues()
{
  path_planning_params_.CADFile = CAD_line_->text().toStdString();
  path_planning_params_.DefectFile = defect_mesh_line_->text().toStdString();
  path_planning_params_.CoveringPercentage = covering_percentage_->value();
  path_planning_params_.ExtricationFrequency = extrication_frequency_->value();
  path_planning_params_.ExtricationCoefficient = extrication_coefficient_->value();
  path_planning_params_.GrindDiameter = grind_diameter_->value();
  path_planning_params_.DepthOfPath = depth_spin_box_->value();
  path_planning_params_.AngleX = lean_angle_axis_x_radio_->isChecked();
  path_planning_params_.AngleY = lean_angle_axis_y_radio_->isChecked();
  path_planning_params_.AngleZ = lean_angle_axis_z_radio_->isChecked();
  path_planning_params_.AngleValue = angle_value_spin_box_->value();
}

void grinding_rviz_plugin::PathPlanningWidget::ComputeTrajectoryButtonHandler()
{
  // Fill service parameters with GUI values
  updateInternalValues();

  // Fill in the request
  srv_path_planning_.request = this->getPathPlanningParams();
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
  if(visualize_trajectory_button_->isEnabled() == false)
  {
    visualize_trajectory_button_->setEnabled(true);
  }
  if(path_planning_simulation_button_->isEnabled() == false)
  {
    path_planning_simulation_button_->setEnabled(true);
  }
  if(generate_trajectory_button_->isEnabled() == false)
  {
    generate_trajectory_button_->setEnabled(true);
  }
}

void grinding_rviz_plugin::PathPlanningWidget::VisualizeTrajectoryButtonHandler()
{
  // If GUI has been changed, compute_trajectory_button_ is enabled.
  // So the pose is not up-to-date with GUI values
  // We inform the user that is an old pose
  if(compute_trajectory_button_->isEnabled() == true)
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

  // Fill in the request
  srv_path_planning_.request = this->getPathPlanningParams();
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
  ROS_ERROR_STREAM("VISUALIZE:");
  ROS_ERROR_STREAM("Compute state" << srv_path_planning_.request.Compute);
  ROS_ERROR_STREAM("Visualization state" << srv_path_planning_.request.Visualization);
  ROS_ERROR_STREAM("Simulation state" << srv_path_planning_.request.Simulation);
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
  if(compute_trajectory_button_->isEnabled() == true)
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

  // Fill in the request
  srv_path_planning_.request = this->getPathPlanningParams();
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
  ROS_ERROR_STREAM("SIMULATE:");
  ROS_ERROR_STREAM("Compute state" << srv_path_planning_.request.Compute);
  ROS_ERROR_STREAM("Visualization state" << srv_path_planning_.request.Visualization);
  ROS_ERROR_STREAM("Simulation state" << srv_path_planning_.request.Simulation);
  path_planning_service_.call(srv_path_planning_);
  // Display return message in Qt panel
  Q_EMIT sendStatus(QString::fromStdString(srv_path_planning_.response.ReturnMessage));

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void grinding_rviz_plugin::PathPlanningWidget::generateTrajectoryButtonHandler()
{
  Q_EMIT enablePanelPostProcessor();
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
  config.mapSetValue(this->objectName() + "CAD_file", CAD_line_->text());
  config.mapSetValue(this->objectName() + "defect_file", defect_mesh_line_->text());
  config.mapSetValue(this->objectName() + "covering_percentage", covering_percentage_->value());
  config.mapSetValue(this->objectName() + "extrication_frequency", extrication_frequency_->value());
  config.mapSetValue(this->objectName() + "extrication_coefficient", extrication_coefficient_->value());
  config.mapSetValue(this->objectName() + "grind_diameter", grind_diameter_->value());
  config.mapSetValue(this->objectName() + "depth_of_path", depth_spin_box_->value());
  config.mapSetValue(this->objectName() + "radio_x", lean_angle_axis_x_radio_->isChecked());
  config.mapSetValue(this->objectName() + "radio_y", lean_angle_axis_y_radio_->isChecked());
  config.mapSetValue(this->objectName() + "radio_z", lean_angle_axis_z_radio_->isChecked());
  config.mapSetValue(this->objectName() + "angle_value", angle_value_spin_box_->value());
}

// Load all configuration data for this panel from the given Config object.
void grinding_rviz_plugin::PathPlanningWidget::load(const rviz::Config& config)
{
  int tmp_int;
  bool tmp_bool;
  float tmp_float;
  QString tmp_string;
  // Load offset value from config file (if it exists)
  if (config.mapGetString(this->objectName() + "CAD_file", &tmp_string))
      CAD_line_->setText(tmp_string);
  if (config.mapGetString(this->objectName() + "defect_file", &tmp_string))
      defect_mesh_line_->setText(tmp_string);
  if (config.mapGetInt(this->objectName() + "covering_percentage", &tmp_int))
      covering_percentage_->setValue(tmp_int);
  if (config.mapGetInt(this->objectName() + "extrication_frequency", &tmp_int))
      extrication_frequency_->setValue(tmp_int);
  if (config.mapGetInt(this->objectName() + "extrication_coefficient", &tmp_int))
      extrication_coefficient_->setValue(tmp_int);
  if (config.mapGetInt(this->objectName() + "grind_diameter", &tmp_int))
      grind_diameter_->setValue(tmp_int);
  if (config.mapGetInt(this->objectName() + "depth_of_path", &tmp_int))
    depth_spin_box_->setValue(tmp_int);
  if (config.mapGetBool(this->objectName() + "radio_x", &tmp_bool))
    lean_angle_axis_x_radio_->setChecked(tmp_bool);
  if (config.mapGetBool(this->objectName() + "radio_y", &tmp_bool))
      lean_angle_axis_y_radio_->setChecked(tmp_bool);
  if (config.mapGetBool(this->objectName() + "radio_z", &tmp_bool))
      lean_angle_axis_z_radio_->setChecked(tmp_bool);
  if (config.mapGetFloat(this->objectName() + "angle_value", &tmp_float))
      angle_value_spin_box_->setValue(tmp_float);
}
