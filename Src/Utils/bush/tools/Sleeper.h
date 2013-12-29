#include <QThread>

class Sleeper : public QThread
{
public:
  static void msleep(unsigned msecs)
  {
    QThread::msleep(msecs);
  }
};
