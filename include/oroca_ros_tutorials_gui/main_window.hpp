/**
 * @file /include/oroca_ros_tutorials_gui/main_window.hpp
 *
 * @brief Qt based gui for oroca_ros_tutorials_gui.
 *
 * @date November 2010
 **/
#ifndef oroca_ros_tutorials_gui_MAIN_WINDOW_H
#define oroca_ros_tutorials_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "msgThread.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace oroca_ros_tutorials_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:


	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

	void showPanel();
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check);
	void on_offboard_button_clicked();
	void on_onboard_button_clicked();
	void on_arm_button_clicked();
	void on_disarm_button_clicked();
	void on_startR_button_clicked();
	void on_stopR_button_clicked();
	void on_startT_button_clicked();
	void on_stopT_button_clicked();
	void on_checkbox_use_environment_stateChanged(int state);
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);
	QNode* returnQnode(){return &qnode;}


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
		void updatePlotPoint(double x,double y, double z);
		void updateRPY(double roll,double pitch,double yaw);
		void updateMODE(std::string mode);
		void updateTARGET(float x,float y,float z);
		void updateNOWXYZ(float x,float y,float z);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	MsgThread msgThread;



};

}  // namespace oroca_ros_tutorials_gui

#endif // oroca_ros_tutorials_gui_MAIN_WINDOW_H
