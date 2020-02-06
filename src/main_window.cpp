/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QWidget>
#include <QMessageBox>
#include <iostream>
#include "../include/oroca_ros_tutorials_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace oroca_ros_tutorials_gui {

using namespace Qt;

#define MOVE_SPD 0.05

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/



// MainWindow 클래스의 생성자
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent), qnode(argc,argv), msgThread(&qnode) // 값 대입
{
	this->setFocusPolicy ( Qt::StrongFocus );


	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)),
                     qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	QObject::connect(ui.actionshow_panel,SIGNAL(triggered(bool)),
									 this, SLOT(showPanel()));

  ReadSettings();
	this->setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()),
                     this, SLOT(close()));

	ui.onboard_button->setEnabled(false); //onboard버튼은 처음에 누르지못함//
	ui.disarm_button->setEnabled(false);
	ui.stopR_button->setEnabled(false);

	ui.startT_button->setEnabled(false);
	ui.stopT_button->setEnabled(false);

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel()); //main_window의 객체를 qnode의 변수에 전달//
    QObject::connect(&qnode, SIGNAL(loggingUpdated()),
                     this, SLOT(updateLoggingView()));//이벤트와 함수 연결//

//rpy 출력
QObject::connect(&qnode, SIGNAL(sendRPY(double,double,double)),
									this, SLOT(updateRPY(double,double,double)));

//그래프에 현재 위치 표시
	QObject::connect(&qnode, SIGNAL(sendPlotPoint(double,double,double)),
										this, SLOT(updatePlotPoint(double,double,double)));
//현재 드론 모드 표시
qRegisterMetaType<std::string>("std::string"); // 기본형이 아닌걸 connect하려면 이렇게 미리 정의를 해줘야 오류 안남!//
	QObject::connect(&qnode, SIGNAL(sendMODE(std::string)),
										this, SLOT(updateMODE(std::string)));
//offboard 모드 상태에서 target x,y,z 표시
QObject::connect(&qnode, SIGNAL(sendTARGET(float,float,float)),
									this, SLOT(updateTARGET(float,float,float)));

//현재 x,y,z 표시
QObject::connect(&qnode, SIGNAL(sendNOWXYZ(float,float,float)),
									this, SLOT(updateNOWXYZ(float,float,float)));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }


		//위치 나타내는 그래프
		ui.plot_xy->addGraph();
		ui.plot_xy->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
		ui.plot_xy->graph(0)->setLineStyle(QCPGraph::lsNone);
		ui.plot_xy->xAxis->setRange(-5,5);
		ui.plot_xy->yAxis->setRange(-5,5);

		//고도 나타내는 그래프
		ui.plot_z->addGraph();
		ui.plot_z->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
		ui.plot_z->graph(0)->setLineStyle(QCPGraph::lsNone);
		ui.plot_z->xAxis->setVisible(false); // x축은 필요없으므로 숨깁니다.
		ui.plot_z->yAxis->setRange(0,10);

}

// 소멸자
MainWindow::~MainWindow() {
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
	if ( ui.checkbox_use_environment->isChecked() ) { // 환경변수를 사용해서 자동으로 ros master에 연결
		if ( !qnode.init() ) { // ros node 시작에 실패하면
			showNoMasterMessage();
		} else { // ros node 시작에 성공하면
			qnode.log(qnode.Info,"connect to ros master successfully");
			ui.button_connect->setEnabled(false);
			msgThread.start();
			qnode.start();
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			qnode.log(qnode.Info,"connect to ros master successfully");
			ui.button_connect->setEnabled(false);
			msgThread.start();
			qnode.start();
		}
	}
}

void MainWindow::on_onboard_button_clicked() {
	qnode.CommandOffboard(false);
	ui.onboard_button->setEnabled(false);
	ui.offboard_button->setEnabled(true);

	ui.target_x->setText("NOT OFFBOARD MODE");
	ui.target_y->setText("NOT OFFBOARD MODE");
	ui.target_z->setText("NOT OFFBOARD MODE");

	ui.startT_button->setEnabled(false);
	ui.stopT_button->setEnabled(false);

}
void MainWindow::on_offboard_button_clicked(){ // offboard 버튼을 누르면 arming 상태로 자동으로 시작합니다.
	qnode.CommandOffboard(true);
	ui.onboard_button->setEnabled(true);
	ui.offboard_button->setEnabled(false);

	qnode.CommandArm(true);
	ui.arm_button->setEnabled(false);
	ui.disarm_button->setEnabled(true);


	ui.startT_button->setEnabled(true);
	ui.stopT_button->setEnabled(false);

}

void MainWindow::on_arm_button_clicked() {
	qnode.CommandArm(true);
	ui.arm_button->setEnabled(false);
	ui.disarm_button->setEnabled(true);
}
void MainWindow::on_disarm_button_clicked(){
	qnode.CommandArm(false);
	ui.arm_button->setEnabled(true);
	ui.disarm_button->setEnabled(false);
}

void MainWindow::on_startR_button_clicked() {
	qnode.startRecord();
	qnode.log(qnode.Info,"start recording");
	ui.startR_button->setEnabled(false);
	ui.stopR_button->setEnabled(true);
}

void MainWindow::on_stopR_button_clicked() {
	qnode.stopRecord();
	qnode.log(qnode.Info,"stop recording");
	ui.startR_button->setEnabled(true);
	ui.stopR_button->setEnabled(false);
}

void MainWindow::on_startT_button_clicked() {
	qnode.startTracking();
	qnode.log(qnode.Info,"start Tracking");
	ui.startT_button->setEnabled(false);
	ui.stopT_button->setEnabled(true);
}

void MainWindow::on_stopT_button_clicked() {
	qnode.stopTracking();
	qnode.log(qnode.Info,"stop Tracking");
	ui.startT_button->setEnabled(true);
	ui.stopT_button->setEnabled(false);
}





void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	/*
	if(event->key() == Qt::Key_W){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setVeloY(FORWARD_SPD);
	}else if(event->key() == Qt::Key_S){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setVeloY(-FORWARD_SPD);
	}else if(event->key() == Qt::Key_D){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setYaw(qnode.changeToRad(-90));
	}else if(event->key() == Qt::Key_A){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setYaw(qnode.changeToRad(90));
	}else if(event->key() == Qt::Key_Up){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setVeloZ(RISE_SPD);
	}else if(event->key() == Qt::Key_Down){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setVeloZ(-RISE_SPD);
	}
	*/
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){
	if(event->key() == Qt::Key_W){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setPosY(MOVE_SPD);
	}else if(event->key() == Qt::Key_S){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setPosY(-MOVE_SPD);
	}else if(event->key() == Qt::Key_D){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setPosX(MOVE_SPD);
	}else if(event->key() == Qt::Key_A){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setPosX(-MOVE_SPD);
	}else if(event->key() == Qt::Key_Up){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setPosZ(MOVE_SPD);
	}else if(event->key() == Qt::Key_Down){
		ROS_INFO("keyPressEvent(%x)", event->key());
		qnode.setPosZ(-MOVE_SPD);
	}
}



/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateRPY(double roll,double pitch,double yaw){
	ui.label_roll->setText(QString::number(roll));
	ui.label_pitch->setText(QString::number(pitch));
	ui.label_yaw->setText(QString::number(yaw));
}




void MainWindow::updatePlotPoint(double x,double y,double z) {
				QVector<double> qv_x,qv_y,qv_z;
				qv_x.append(x);
				qv_y.append(y);
				qv_z.append(z);
        ui.plot_xy->graph(0)->setData(qv_x,qv_y);
				ui.plot_xy->replot();
				ui.plot_xy->update();

				QVector<double> qv_zx; // 고도 그래프에서 점을 중간에 위치시키기 위한 x좌표
				qv_zx.append(2.5);
				ui.plot_z->graph(0)->setData(qv_zx,qv_z);
				ui.plot_z->replot();
				ui.plot_z->update();
}


void MainWindow::updateMODE(std::string mode) {
	ui.label_mode->setText(QString::fromStdString(mode));
}

void MainWindow::updateTARGET(float x,float y,float z){
	ui.target_x->setText(QString::number(x));
	ui.target_y->setText(QString::number(y));
	ui.target_z->setText(QString::number(z));
}

void MainWindow::updateNOWXYZ(float x,float y,float z){
	ui.now_x->setText(QString::number(x));
	ui.now_y->setText(QString::number(y));
	ui.now_z->setText(QString::number(z));
}



/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "oroca_ros_tutorials_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::showPanel(){
	ui.dock_status->show();
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "oroca_ros_tutorials_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace oroca_ros_tutorials_gui
