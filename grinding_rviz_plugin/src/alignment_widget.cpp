
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QRadioButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QFuture>
#include <QtConcurrentRun>
#include <QFont>

#include "alignment_widget.h"

grinding_rviz_plugin::AlignmentWidget::AlignmentWidget(QWidget* parent) : QWidget(parent)
{
  this->setObjectName("AlignmentWidget_");

  uniform_sampling_label_ = new QLabel;
  uniform_sampling_label_->setText("Uniform sampling for CAD mesh:");
  QFont bold_font;
  bold_font.setBold(true);
  uniform_sampling_label_->setFont(bold_font);
  number_samples_label_ = new QLabel;
  number_samples_label_->setText("Default samples number");
  number_samples_ = new QSpinBox;
  number_samples_->setMinimum(1);
  number_samples_->setMaximum(1000000);
  QHBoxLayout* number_samples_layout = new QHBoxLayout;
  number_samples_layout->addWidget(number_samples_label_);
  number_samples_layout->addWidget(number_samples_);
  QVBoxLayout* uniform_sampling_layout = new QVBoxLayout;
  uniform_sampling_layout->addWidget(uniform_sampling_label_);
  uniform_sampling_layout->addLayout(number_samples_layout);
  uniform_sampling_layout->addStretch(1);

  voxel_grid_label_ = new QLabel;
  voxel_grid_label_->setText("Set a voxel grid on meshes:");
  voxel_grid_label_->setFont(bold_font);
  leaf_size_label_ = new QLabel;
  leaf_size_label_->setText("Leaf size (in mm3):");
  leaf_size_ = new QDoubleSpinBox;
  leaf_size_->setMinimum(0.01);
  leaf_size_->setMaximum(1);
  QHBoxLayout* leaf_size_layout = new QHBoxLayout;
  leaf_size_layout->addWidget(leaf_size_label_);
  leaf_size_layout->addWidget(leaf_size_);
  QVBoxLayout* voxel_grid_layout = new QVBoxLayout;
  voxel_grid_layout->addWidget(voxel_grid_label_);
  voxel_grid_layout->addLayout(leaf_size_layout);
  voxel_grid_layout->addStretch(1);

  icp_label_ = new QLabel;
  icp_label_->setText("ICP process:");
  icp_label_->setFont(bold_font);
  iteration_number_label_ = new QLabel;
  iteration_number_label_->setText("Number of ICP iterations:");
  iteration_number_ = new QSpinBox;
  iteration_number_->setMinimum(1);
  iteration_number_->setMaximum(5000);
  QHBoxLayout* iteration_number_layout = new QHBoxLayout;
  iteration_number_layout->addWidget(iteration_number_label_);
  iteration_number_layout->addWidget(iteration_number_);
  max_iteration_number_label_ = new QLabel;
  max_iteration_number_label_->setText("Maximum number of iterations");
  max_iteration_number_ = new QSpinBox;
  max_iteration_number_->setMinimum(1);
  max_iteration_number_->setMaximum(5000);
  QHBoxLayout* max_iteration_number_layout = new QHBoxLayout;
  max_iteration_number_layout->addWidget(max_iteration_number_label_);
  max_iteration_number_layout->addWidget(max_iteration_number_);
  max_corr_dist_label_ = new QLabel;
  max_corr_dist_label_->setText("Max correspondence distance:");
  max_corr_dist_ = new QDoubleSpinBox;
  max_corr_dist_->setMinimum(0.01);
  max_corr_dist_->setMaximum(5);
  QHBoxLayout* max_corr_dist_layout = new QHBoxLayout;
  max_corr_dist_layout->addWidget(max_corr_dist_label_);
  max_corr_dist_layout->addWidget(max_corr_dist_);
  QVBoxLayout* icp_layout = new QVBoxLayout;
  icp_layout->addWidget(icp_label_);
  icp_layout->addLayout(iteration_number_layout);
  icp_layout->addLayout(max_iteration_number_layout);
  icp_layout->addLayout(max_corr_dist_layout);
  icp_layout->addStretch(1);

  alignment_button_ = new QPushButton;
  alignment_button_->setText("Start Alignment");

  QVBoxLayout* alignment_layout = new QVBoxLayout(this);
  alignment_layout->addLayout(uniform_sampling_layout);
  alignment_layout->addLayout(voxel_grid_layout);
  alignment_layout->addLayout(icp_layout);
  alignment_layout->addWidget(alignment_button_);

  // Connect handlers
  connect(number_samples_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(leaf_size_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
  connect(iteration_number_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(max_iteration_number_, SIGNAL(valueChanged(int)), this, SLOT(triggerSave()));
  connect(max_corr_dist_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));

  connect(alignment_button_, SIGNAL(released()), this, SLOT(AlignmentButtonHandler()));

  //Setup client
  alignment_service_ = nh_.serviceClient<alignment::AlignmentService>("alignment_service");
}

void grinding_rviz_plugin::AlignmentWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
  updateGUI();
}

alignment::AlignmentService::Request grinding_rviz_plugin::AlignmentWidget::getAlignmentParams()
{
  return alignment_params_;
}

void grinding_rviz_plugin::AlignmentWidget::setAlignmentParams(alignment::AlignmentService::Request params)
{
  alignment_params_.CADFileName = params.CADFileName;
  alignment_params_.CADMarkerName = params.CADMarkerName;
  alignment_params_.ScanFileName = params.ScanFileName;
  alignment_params_.ScanMarkerName = params.ScanMarkerName;
  alignment_params_.NumberUniformSampling = params.NumberUniformSampling;
  alignment_params_.LeafSize = params.LeafSize;
  alignment_params_.IterationNumber = params.IterationNumber;
  alignment_params_.MaxIterationNumber = params.IterationNumber;
  alignment_params_.MaxCorrDist = params.MaxCorrDist;
  updateGUI();
}

void grinding_rviz_plugin::AlignmentWidget::updateGUI()
{
  number_samples_->setValue(alignment_params_.NumberUniformSampling);
  leaf_size_->setValue(alignment_params_.LeafSize);
  iteration_number_->setValue(alignment_params_.IterationNumber);
  max_iteration_number_->setValue(alignment_params_.MaxIterationNumber);
  max_corr_dist_->setValue(alignment_params_.MaxCorrDist);
}

void grinding_rviz_plugin::AlignmentWidget::updateInternalValues()
{
  alignment_params_.NumberUniformSampling = number_samples_->value();
  alignment_params_.LeafSize = leaf_size_->value();
  alignment_params_.IterationNumber = iteration_number_->value();
  alignment_params_.MaxIterationNumber = max_iteration_number_->value();
  alignment_params_.MaxCorrDist = max_corr_dist_->value();
}

void grinding_rviz_plugin::AlignmentWidget::setCADAndScanParams(const QString cad_filename,
                                                                const QString cad_marker_name,
                                                                const QString scan_filename,
                                                                const QString scan_marker_name)
{
  alignment_params_.CADFileName = cad_filename.toStdString();
  alignment_params_.CADMarkerName = cad_marker_name.toStdString();
  alignment_params_.ScanFileName = scan_filename.toStdString();
  alignment_params_.ScanMarkerName = scan_marker_name.toStdString();
}

void grinding_rviz_plugin::AlignmentWidget::AlignmentButtonHandler()
{
  // get CAD and Scan params which are stored in grinding rviz plugin
  Q_EMIT getCADAndScanParams();
  // Fill in the request
  srv_alignment_.request = getAlignmentParams();
  //srv_.request.*request* = *value*;
  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &AlignmentWidget::Alignment);
}

void grinding_rviz_plugin::AlignmentWidget::Alignment()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service
  alignment_service_.call(srv_alignment_);
  Q_EMIT sendStatus(QString::fromStdString(srv_alignment_.response.ReturnMessage));

  if(srv_alignment_.response.ReturnStatus == true)
    Q_EMIT enablePanelComparison();
  else
  {
    Q_EMIT sendMsgBox("Error aligning meshes",
                      QString::fromStdString(srv_alignment_.response.ReturnMessage), "");
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void grinding_rviz_plugin::AlignmentWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT sendStatus("Connecting to service");
  while (ros::ok())
  {
    if (alignment_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel connected to the service " << alignment_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + alignment_service_.getService()));
      break;
    }
    else
    {
      ROS_ERROR_STREAM("RViz panel could not connect to ROS service:\n\t" << alignment_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + alignment_service_.getService()));
      sleep(1);
    }
  }

  ROS_WARN_STREAM("Service connection have been made");
  Q_EMIT sendStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

// Save all configuration data from this panel to the given Config object
void grinding_rviz_plugin::AlignmentWidget::save(rviz::Config config)
{
  config.mapSetValue(this->objectName() + "number_uniform_sampling", number_samples_->value());
  config.mapSetValue(this->objectName() + "leaf_size_voxel_grid", leaf_size_->value());
  config.mapSetValue(this->objectName() + "number_iteration_icp", iteration_number_->value());
  config.mapSetValue(this->objectName() + "max_number_iteration_icp", max_iteration_number_->value());
  config.mapSetValue(this->objectName() + "max_corr_dist_icp", max_corr_dist_->value());
}

// Load all configuration data for this panel from the given Config object.
void grinding_rviz_plugin::AlignmentWidget::load(const rviz::Config& config)
{
  float float_tmp;
  int int_tmp;

  if (config.mapGetInt(this->objectName() + "number_uniform_sampling", &int_tmp))
    number_samples_->setValue(int_tmp);
  else
  {
    number_samples_->setValue(100000);
  }
  if (config.mapGetFloat(this->objectName() + "leaf_size_voxel_grid", &float_tmp))
    leaf_size_->setValue(float_tmp);
  else
  {
    leaf_size_->setValue(0.02);
  }
  if (config.mapGetInt(this->objectName() + "number_iteration_icp", &int_tmp))
    iteration_number_->setValue(int_tmp);
  else
  {
    iteration_number_->setValue(100);
  }
  if (config.mapGetInt(this->objectName() + "max_number_iteration_icp", &int_tmp))
    max_iteration_number_->setValue(int_tmp);
  else
  {
    max_iteration_number_->setValue(500);
  }
  if (config.mapGetFloat(this->objectName() + "max_corr_dist_icp", &float_tmp))
    max_corr_dist_->setValue(float_tmp);
  else
  {
    max_corr_dist_->setValue(0.5);
  }
}
