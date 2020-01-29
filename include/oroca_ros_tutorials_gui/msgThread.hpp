#include <QThread>
#include "qnode.hpp"

namespace oroca_ros_tutorials_gui {

class MsgThread : public QThread
{
  Q_OBJECT //왜 넣는지 나중에 알아보기
public:
  MsgThread(QNode* q);
  virtual ~MsgThread();
  void run();

Q_SIGNALS:
  void finish();
private:
  QNode* qnode;


};

}
