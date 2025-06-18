#include <QApplication>
#include "mainwindow.h"
#include <bamboo/application/application.h>
using namespace welkin;

int main(int argc, char *argv[]) {
    bamboo::Application a(argc, argv);
    // a.getParamList()->add("uface_title", "BambooTerrain");
    // a.getParamList()->add("uface_icon_file", "bamboo_terrain.png");
    // a.getParamList()->add("uface_theme", "light");
    // a.getParamList()->add("uface_translations[stringlist]: (bamboo)");
    // a.getParamList()->add("uface_language", "zh_CN");
    a.initSheetStyle();
    a.initTranslations();

    bamboo::MainWindow mw;
    mw.init(a.getParamList());
    a.installHelpMenu();
    mw.installMessagePanel(Qt::LeftDockWidgetArea);
    mw.installMenus();
    // mw.show();
    mw.showMaximized();
    return a.exec();
}
