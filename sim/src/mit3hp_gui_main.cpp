#include "Mit3hpControlPanel.h"
#include <QApplication>

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  Mit3hpControlPanel panel;
  panel.show();
  return app.exec();
}
