/**
 * @file /include/oroca_ros_tutorials_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef oroca_ros_tutorials_gui_QNODE_HPP_
#define oroca_ros_tutorials_gui_QNODE_HPP_



/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <fstream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace oroca_ros_tutorials_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  void setPosZ(float a);
  void setPosX(float a);
  void setPosY(float a);
  void setYaw(float a);
  float changeToRad(float a);
  float ChangeToDegree(float a);
  void myLocationLogging();
	void clearLogging();
  void myRPYLogging();
  void MODELogging(std::string mode);
  void TARGETLogging(float x,float y,float z);
  void inputToTextfile();
  void CommandArm(bool boolean);
  void CommandOffboard(bool boolean);
  void startRecord();
  void stopRecord();


Q_SIGNALS:
	  void loggingUpdated();
    void rosShutdown();
    void sendPlotPoint(double x,double y,double z);
    void sendRPY(double roll, double pitch, double yaw);
    void sendMODE(std::string mode);
    void sendTARGET(float x,float y,float z);
    void sendNOWXYZ(float x,float y,float z);


private:
	int init_argc;
	char** init_argv;

  bool offboard;
  bool arming;
  ros::Subscriber state_sub;
  ros::Subscriber position_sub;
  ros::Subscriber velocity_sub;
  ros::ServiceClient arming_client;
  ros::ServiceClient land_client;
  ros::Publisher local_pos_pub;
  ros::ServiceClient set_mode_client;
  QStringListModel logging_model;



  //텍스트 파일 저장 관련 변수
  std::string filePath;
  std::ofstream writeFile;

  bool bool_recoding;
  // 끝




};

}  // namespace oroca_ros_tutorials_gui

#endif /* oroca_ros_tutorials_gui_QNODE_HPP_ */
