#ifndef ALIGNMENT_WIDGET_H
#define ALIGNMENT_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <fanuc_grinding_alignment/AlignmentService.h>
#endif

class QLabel;
class QVBoxLayout;
class QPushButton;

namespace fanuc_grinding_rviz_plugin
{
class AlignmentWidget : public QWidget
{
  Q_OBJECT
public:
  AlignmentWidget(QWidget* parent =  NULL);
  void load(const rviz::Config& config);
  void save(rviz::Config config);
  void setAlignmentParams(const fanuc_grinding_alignment::AlignmentService::Request &params);

Q_SIGNALS:
  void guiChanged();
  void sendStatus(const QString status);
  void sendMsgBox(const QString title, const QString msg, const QString info_msg);
  void enablePanel(const bool);
  void enablePanelComparison();
  void getCADAndScanParams();

protected Q_SLOTS:
  void connectToServices();
  void triggerSave();
  void updateGUI();
  void updateInternalValues();
  void alignmentButtonHandler();
  void alignment();
  void enablePanelComparisonHandler();
  void setCADAndScanParams(const QString cad_filename,
                           const QString scan_filename);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient alignment_service_;
  fanuc_grinding_alignment::AlignmentService srv_alignment_;

  QPushButton* alignment_button_;
};

} // End namespace

#endif // ALIGNMENT_WIDGET_H
