
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QFuture>
#include <QtConcurrentRun>

#include "post_processor_widget.h"

grinding_rviz_plugin::PostProcessorWidget::PostProcessorWidget(QWidget* parent) : QWidget(parent)
{
  this->setObjectName("PostProcessorWidget_");

  program_name_label_ = new QLabel;
  program_name_label_->setText("Program name: ");
  program_name_ = new QLineEdit;
  program_name_layout_ = new QHBoxLayout;
  program_name_layout_->addWidget(program_name_label_);
  program_name_layout_->addWidget(program_name_);

  ip_adress_label_ = new QLabel;
  ip_adress_label_->setText("Robot IP: ");
  ip_adress_ = new QLineEdit;
  ip_adress_->setInputMask("000.000.000.000;_"); // FIXME: Take a look on this line
  ip_adress_->setText("192.168.100.200");
  ip_adress_layout_ = new QHBoxLayout;
  ip_adress_layout_->addWidget(ip_adress_label_);
  ip_adress_layout_->addWidget(ip_adress_);

  program_location_label_ = new QLabel;
  program_location_label_->setText("Program location :");
  browse_program_location_button_ = new QPushButton;
  browse_program_location_button_->setText("Browse...");
  program_location_ = new QLineEdit;
  program_location_->setReadOnly(true);
  program_location_layout_ = new QHBoxLayout;
  program_location_layout_->addWidget(program_location_label_);
  program_location_layout_->addWidget(browse_program_location_button_);

  comment_label_ = new QLabel;
  comment_label_->setText("Comment (Optional) :");
  comment_ = new QLineEdit;
  comment_layout_ = new QHBoxLayout();
  comment_layout_->addWidget(comment_label_);
  comment_layout_->addWidget(comment_);

  generate_program_button_ = new QPushButton;
  generate_program_button_->setText("Generate TP program");

  post_processor_layout_ = new QVBoxLayout(this);
  post_processor_layout_->addLayout(program_name_layout_);
  post_processor_layout_->addStretch(10);
  post_processor_layout_->addLayout(ip_adress_layout_);
  post_processor_layout_->addStretch(10);
  post_processor_layout_->addLayout(program_location_layout_);
  post_processor_layout_->addWidget(program_location_);
  post_processor_layout_->addStretch(10);
  post_processor_layout_->addLayout(comment_layout_);
  post_processor_layout_->addStretch(10);
  post_processor_layout_->addWidget(generate_program_button_);

  // Connect Handlers
  connect(program_name_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(ip_adress_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(program_location_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(comment_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(browse_program_location_button_, SIGNAL(released()), this, SLOT(browseProgramLocation()));
  connect(generate_program_button_, SIGNAL(released()), this, SLOT(GenerateProgramButtonHandler()));

  connect(program_name_, SIGNAL(editingFinished()), this, SLOT(tweakProgramName()));

  //Setup client
  post_processor_service_ = post_processor_node_.serviceClient<post_processor::PostProcessorService>("post_processor_service");

  QFuture<void> future = QtConcurrent::run(this, &grinding_rviz_plugin::PostProcessorWidget::connectToServices);
}

void grinding_rviz_plugin::PostProcessorWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
  updateGUI();
}

post_processor::PostProcessorService::Request grinding_rviz_plugin::PostProcessorWidget::getPostProcessorParams()
{
  return post_processor_params_;
}

void grinding_rviz_plugin::PostProcessorWidget::setPostProcessorParams(post_processor::PostProcessorService::Request params)
{
  post_processor_params_.ProgramName=params.ProgramName;
  post_processor_params_.IpAdress=params.IpAdress;
  post_processor_params_.ProgramLocation=params.ProgramLocation;
  post_processor_params_.Comment=params.Comment;
  updateGUI();
}

void grinding_rviz_plugin::PostProcessorWidget::updateGUI()
{
  program_name_->setText(QString::fromStdString(post_processor_params_.ProgramName));
  ip_adress_->setText(QString::fromStdString(post_processor_params_.IpAdress));
  program_location_->setText(QString::fromStdString(post_processor_params_.ProgramLocation));
  comment_->setText(QString::fromStdString(post_processor_params_.Comment));
}

void grinding_rviz_plugin::PostProcessorWidget::updateInternalValues()
{
  post_processor_params_.ProgramName = program_name_->text().toStdString();
  post_processor_params_.IpAdress = ip_adress_->text().toStdString();
  post_processor_params_.ProgramLocation = program_location_->text().toStdString();
  post_processor_params_.Comment = comment_->text().toStdString();
}

void grinding_rviz_plugin::PostProcessorWidget::tweakProgramName()
{
  if (program_name_->text().endsWith("."))
    program_name_->setText(program_name_->text().append("ls"));

  if (program_name_->text().endsWith(".l"))
    program_name_->setText(program_name_->text().append("s"));

  if (!program_name_->text().contains(".ls"))
    program_name_->setText(program_name_->text().append(".ls"));
}

void grinding_rviz_plugin::PostProcessorWidget::setRobotPoses(std::vector<geometry_msgs::Pose> robot_poses)
{
  for(std::vector<geometry_msgs::Pose>::iterator iter (robot_poses.begin());
      iter != robot_poses.end();
      ++iter)
  {
    post_processor_params_.RobotPoses.push_back(*iter);
  }
}

void grinding_rviz_plugin::PostProcessorWidget::setPointColorViz(std::vector<bool> point_color_viz)
{
  for(std::vector<bool>::iterator iter (point_color_viz.begin());
        iter != point_color_viz.end();
        ++iter)
  {
    post_processor_params_.PointColorViz.push_back(*iter);
  }
}

void grinding_rviz_plugin::PostProcessorWidget::setIndexVector(std::vector<int> index_vector)
{
  for(std::vector<int>::iterator iter (index_vector.begin());
        iter != index_vector.end();
        ++iter)
  {
    post_processor_params_.IndexVector.push_back(*iter);
  }
}

void grinding_rviz_plugin::PostProcessorWidget::browseProgramLocation()
{
  QFileDialog program_location_browser;

  program_location_->setText(program_location_browser.getExistingDirectory(0, tr("Select file location"), "/home/dell"));
}

void grinding_rviz_plugin::PostProcessorWidget::GenerateProgramButtonHandler()
{
  if(program_name_->text().isEmpty() || ip_adress_->text().isEmpty() || program_location_->text().isEmpty())
  {
    Q_EMIT sendStatus("Program name, Robot IP address and program location fields have to be filled");
  }
  else
  {
    // Fill in the request
    Q_EMIT getRobotPosesData();
    srv_post_processor_.request = this->getPostProcessorParams();
    //srv_.request.*request* = *value*;
    // Start client service call in an other thread
    QFuture<void> future = QtConcurrent::run(this, &PostProcessorWidget::GenerateProgram);
  }
}

void grinding_rviz_plugin::PostProcessorWidget::GenerateProgram()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service
  post_processor_service_.call(srv_post_processor_);
  Q_EMIT sendStatus(QString::fromStdString(srv_post_processor_.response.ReturnMessage));

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void grinding_rviz_plugin::PostProcessorWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT sendStatus("Connecting to service");
  while (ros::ok())
  {
    if (post_processor_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel connected to the service " << post_processor_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + post_processor_service_.getService()));
      break;
    }
    else
    {
      ROS_ERROR_STREAM("RViz panel could not connect to ROS service:\n\t" << post_processor_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + post_processor_service_.getService()));
      sleep(1);
    }
  }

  ROS_WARN_STREAM("Service connection have been made");
  Q_EMIT sendStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

// Save all configuration data from this panel to the given Config object
void grinding_rviz_plugin::PostProcessorWidget::save(rviz::Config config)
{
  // Save offset value into the config file
  config.mapSetValue(this->objectName() + "program_name", program_name_->text());
  config.mapSetValue(this->objectName() + "ip_adress", ip_adress_->text());
  config.mapSetValue(this->objectName() + "program_location", program_location_->text());
  config.mapSetValue(this->objectName() + "comment", comment_->text());
}

// Load all configuration data for this panel from the given Config object.
void grinding_rviz_plugin::PostProcessorWidget::load(const rviz::Config& config)
{
  QString tmp;
  // Load offset value from config file (if it exists)
  if (config.mapGetString(this->objectName() + "program_name", &tmp))
    program_name_->setText(tmp);
  if (config.mapGetString(this->objectName() + "ip_adress", &tmp))
    ip_adress_->setText(tmp);
  if (config.mapGetString(this->objectName() + "program_location", &tmp))
    program_location_->setText(tmp);
  if (config.mapGetString(this->objectName() + "comment", &tmp))
    comment_->setText(tmp);
}
