#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QFuture>
#include <QtConcurrentRun>
#include <QSpinBox>

#include "post_processor_widget.h"

fanuc_grinding_rviz_plugin::PostProcessorWidget::PostProcessorWidget(QWidget* parent) : QWidget(parent)
{
  setObjectName("PostProcessorWidget_");

  QLabel* program_name_label = new QLabel("Program name:");
  program_name_ = new QLineEdit;
  QHBoxLayout* program_name_layout = new QHBoxLayout;
  program_name_layout->addWidget(program_name_label);
  program_name_layout->addWidget(program_name_);

  QLabel* comment_label = new QLabel("Comment (optional):");
  comment_ = new QLineEdit;
  QHBoxLayout* comment_layout_ = new QHBoxLayout();
  comment_layout_->addWidget(comment_label);
  comment_layout_->addWidget(comment_);

  QLabel* machining_speed_label = new QLabel("Machining speed (cm/min): ");
  machining_speed_ = new QSpinBox;
  machining_speed_->setRange(1, 1000);
  QHBoxLayout* machining_speed_layout_ = new QHBoxLayout();
  machining_speed_layout_->addWidget(machining_speed_label);
  machining_speed_layout_->addWidget(machining_speed_);

  QLabel* extrication_speed_label = new QLabel("Extrication speed (cm/min): ");
  extrication_speed_ = new QSpinBox;
  extrication_speed_->setRange(1, 1000);
  QHBoxLayout* extrication_speed_layout_ = new QHBoxLayout();
  extrication_speed_layout_->addWidget(extrication_speed_label);
  extrication_speed_layout_->addWidget(extrication_speed_);

  QLabel* upload_label = new QLabel("Upload program:");
  upload_program_ = new QCheckBox;
  QHBoxLayout* upload_layout_ = new QHBoxLayout();
  upload_layout_->addWidget(upload_label);
  upload_layout_->addStretch(1);
  upload_layout_->addWidget(upload_program_);
  upload_layout_->addStretch(9);

  ip_adress_label_ = new QLabel("Robot IP:");
  ip_address_ = new QLineEdit;
  ip_address_->setInputMask("000.000.000.000;_");
  ip_address_->setText("192.168.100.200");
  QHBoxLayout* ip_adress_layout = new QHBoxLayout;
  ip_adress_layout->addWidget(ip_adress_label_);
  ip_adress_layout->addWidget(ip_address_);

  QLabel* program_location_label = new QLabel("Program location:");
  program_location_ = new QLineEdit;
  program_location_->setReadOnly(true);
  QHBoxLayout* program_location_layout = new QHBoxLayout;
  program_location_layout->addWidget(program_location_label);
  program_location_layout->addWidget(program_location_);

  generate_program_button_ = new QPushButton("Generate TP program");
  generate_program_button_->setMinimumHeight(90);

  QVBoxLayout* post_processor_layout = new QVBoxLayout(this);
  post_processor_layout->addLayout(program_name_layout);
  post_processor_layout->addLayout(comment_layout_);
  post_processor_layout->addLayout(machining_speed_layout_);
  post_processor_layout->addLayout(extrication_speed_layout_);
  post_processor_layout->addLayout(upload_layout_);
  post_processor_layout->addLayout(ip_adress_layout);
  post_processor_layout->addLayout(program_location_layout);
  post_processor_layout->addStretch(2);
  post_processor_layout->addWidget(generate_program_button_);
  post_processor_layout->addStretch(8);

  // Connect Handlers
  connect(program_name_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(comment_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(machining_speed_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(extrication_speed_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(upload_program_, SIGNAL(stateChanged(int)), this, SLOT(setIpAddressEnable(int)));
  connect(upload_program_, SIGNAL(stateChanged(int)), this, SLOT(triggerSave()));
  connect(ip_address_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(generate_program_button_, SIGNAL(released()), this, SLOT(generateProgramButtonHandler()));
  connect(program_name_, SIGNAL(editingFinished()), this, SLOT(tweakProgramName()));
  tweakProgramName();

  // Program location is hard-coded
  setProgramLocation(ros::package::getPath("fanuc_grinding_rviz_plugin")+"/tp_programs/");

  //Setup client
  post_processor_service_ = post_processor_node_.serviceClient<fanuc_grinding_post_processor::PostProcessorService>("post_processor_service");

  QFuture<void> future = QtConcurrent::run(this, &fanuc_grinding_rviz_plugin::PostProcessorWidget::connectToServices);
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::setPostProcessorParams(const fanuc_grinding_post_processor::PostProcessorService::Request &params)
{
  // Copy all BUT trajectory data
  srv_post_processor_.request.ProgramLocation = params.ProgramLocation;
  srv_post_processor_.request.ProgramName = params.ProgramName;
  srv_post_processor_.request.Comment = params.Comment;
  srv_post_processor_.request.MachiningSpeed = params.MachiningSpeed;
  srv_post_processor_.request.ExtricationSpeed = params.ExtricationSpeed;
  srv_post_processor_.request.Upload = params.Upload;
  srv_post_processor_.request.IpAdress = params.IpAdress;
  // Probably not filled
  srv_post_processor_.request.RobotPoses = params.RobotPoses;
  srv_post_processor_.request.PointColorViz = params.PointColorViz;
  srv_post_processor_.request.IndexVector = params.IndexVector;
  updateGUI();
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::updateGUI()
{
  program_name_->setText(QString::fromStdString(srv_post_processor_.request.ProgramName));
  comment_->setText(QString::fromStdString(srv_post_processor_.request.Comment));
  machining_speed_->setValue(srv_post_processor_.request.MachiningSpeed);
  extrication_speed_->setValue(srv_post_processor_.request.ExtricationSpeed);
  upload_program_->setChecked(srv_post_processor_.request.Upload);
  ip_address_->setText(QString::fromStdString(srv_post_processor_.request.IpAdress));
  program_location_->setText(QString::fromStdString(srv_post_processor_.request.ProgramLocation));
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::updateInternalValues()
{
  srv_post_processor_.request.ProgramName = program_name_->text().toStdString();
  srv_post_processor_.request.Comment = comment_->text().toStdString();
  srv_post_processor_.request.MachiningSpeed = machining_speed_->value();
  srv_post_processor_.request.ExtricationSpeed = extrication_speed_->value();
  srv_post_processor_.request.Upload = upload_program_->isChecked();
  srv_post_processor_.request.IpAdress = ip_address_->text().toStdString();
  // program_location_ is read only
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::tweakProgramName()
{
  if (program_name_->text().size() == 0)
    return;

  if (program_name_->text().endsWith("."))
    program_name_->setText(program_name_->text().append("ls"));

  if (program_name_->text().endsWith(".l"))
    program_name_->setText(program_name_->text().append("s"));

  if (!program_name_->text().contains(".ls"))
    program_name_->setText(program_name_->text().append(".ls"));
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::setIpAddressEnable(const int state)
{
  ip_address_->setEnabled(state);
  ip_adress_label_->setEnabled(state);
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::setProgramLocation(const std::string &location)
{
  srv_post_processor_.request.ProgramLocation = location;
  updateGUI();
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::setRobotPoses(const std::vector<geometry_msgs::Pose> &robot_poses)
{
  srv_post_processor_.request.RobotPoses.clear();
  for(std::vector<geometry_msgs::Pose>::const_iterator iter (robot_poses.begin());
      iter != robot_poses.end();
      ++iter)
  {
    srv_post_processor_.request.RobotPoses.push_back(*iter);
  }
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::setPointColorViz(const std::vector<bool> &point_color_viz)
{
  srv_post_processor_.request.PointColorViz.clear();
  for(std::vector<bool>::const_iterator iter (point_color_viz.begin());
        iter != point_color_viz.end();
        ++iter)
  {
    srv_post_processor_.request.PointColorViz.push_back(*iter);
  }
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::setIndexVector(const std::vector<int> &index_vector)
{
  srv_post_processor_.request.IndexVector.clear();
  for(std::vector<int>::const_iterator iter (index_vector.begin());
        iter != index_vector.end();
        ++iter)
  {
    srv_post_processor_.request.IndexVector.push_back(*iter);
  }
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::generateProgramButtonHandler()
{
  updateInternalValues();
  Q_EMIT getRobotTrajectoryData();

  if (srv_post_processor_.request.ProgramName.empty())
  {
    Q_EMIT sendMsgBox("Program name",
                      "The program name cannot be empty", "");
    Q_EMIT sendStatus("Program name is empty");
    return;
  }

  // Request has been filled with updateInternalValues and getRobotTrajectoryData();
  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &PostProcessorWidget::generateProgram);
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::generateProgram()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service
  post_processor_service_.call(srv_post_processor_);

  if(srv_post_processor_.response.ReturnStatus == true)
  {
    Q_EMIT sendStatus(QString::fromStdString(srv_post_processor_.response.ReturnMessage));
  }
  else
  {
    Q_EMIT sendMsgBox("Error in post-processor",
                      QString::fromStdString(srv_post_processor_.response.ReturnMessage),
                      "");
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void fanuc_grinding_rviz_plugin::PostProcessorWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT sendStatus("Connecting to service");
  while (ros::ok())
  {
    if (post_processor_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(objectName().toStdString() + " RViz panel connected to the service " << post_processor_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + post_processor_service_.getService()));
      break;
    }
    else
    {
      ROS_WARN_STREAM(objectName().toStdString() + " RViz panel could not connect to ROS service:\n\t" << post_processor_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + post_processor_service_.getService()));
      sleep(1);
    }
  }

  ROS_INFO_STREAM(objectName().toStdString() + " service connections have been made");
  Q_EMIT sendStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

// Save all configuration data from this panel to the given Config object
void fanuc_grinding_rviz_plugin::PostProcessorWidget::save(rviz::Config config)
{
  // Save offset value into the config file
  config.mapSetValue(objectName() + "program_name", program_name_->text());
  config.mapSetValue(objectName() + "comment", comment_->text());
  config.mapSetValue(objectName() + "machining_speed", machining_speed_->value());
  config.mapSetValue(objectName() + "extrication_speed", extrication_speed_->value());
  config.mapSetValue(objectName() + "upload_program", upload_program_->isChecked());
  config.mapSetValue(objectName() + "ip_adress", ip_address_->text());
}

// Load all configuration data for this panel from the given Config object.
void fanuc_grinding_rviz_plugin::PostProcessorWidget::load(const rviz::Config& config)
{
  QString tmp;
  float int_tmp;
  // Load offset value from config file (if it exists)
  if (config.mapGetString(objectName() + "program_name", &tmp))
    program_name_->setText(tmp);
  if (config.mapGetString(objectName() + "comment", &tmp))
    comment_->setText(tmp);

  if (config.mapGetFloat(objectName() + "machining_speed", &int_tmp))
    machining_speed_->setValue(int_tmp);
  else
    machining_speed_->setValue(200);

  if (config.mapGetFloat(objectName() + "extrication_speed", &int_tmp))
    extrication_speed_->setValue(int_tmp);
  else
    extrication_speed_->setValue(500);

  bool state_tmp;
  if (config.mapGetBool(objectName() + "upload_program", &state_tmp))
      upload_program_->setChecked(state_tmp);
  setIpAddressEnable(upload_program_->isChecked());

  if (config.mapGetString(objectName() + "ip_adress", &tmp))
    ip_address_->setText(tmp);

  updateInternalValues();
}
