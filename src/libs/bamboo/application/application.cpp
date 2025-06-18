#include "application.h"
#include <QDebug>
#include <osg/GL>
#include <QColorDialog>
#include <QSurfaceFormat>
#include <common/base/dir.h>
#include <uface/base/uconverter.h>
#include <uface/mainwindow/ustylesheet.h>
#include <uface/mainwindow/utranslator.h>
#include <uface/interface/imainwindow.h>
namespace welkin::bamboo {

Application::Application(int &argc, char **argv)
        : QApplication(argc, argv) {
#ifdef OS_WINDOWS
    QString conf_path = "C:/welkin/conf";
#else
    QString conf_path = "/welkin/panda/conf";
#endif
    if (!common::Dir(UFQ(conf_path)).isExist()) {
        QString app_path = QCoreApplication::applicationDirPath();
        conf_path = app_path + "/conf";
    }
    std::string app_name = common::Dir(std::string(argv[0])).getFileName();
    // 日志目录
    std::string log_dir = UFQ(conf_path) + "/" + app_name;
    common::GLogger glogger(log_dir, argv[0]);

    qDebug() << "Conf Path: " << conf_path;
    std::string common_conf_file = UFQ(conf_path) + "/common.conf";
    if (!_param_list.loadFromFile(common_conf_file)) {
        AWARN << "Cannot open common config file: " << common_conf_file;
    }

    std::string conf_file = UFQ(conf_path) + "/" + app_name + ".conf";
    if (!_param_list.loadFromFile(conf_file)) {
        AERROR << "Cannot open config file: " << conf_file;
    }
    _param_list.add("conf_path", UFQ(conf_path));

    _ssht = new UStyleSheet(nullptr);
    _trsl = new UTranslator(nullptr);

    ///
    QColorDialog::setCustomColor(0, QColor(255, 255, 255));
    QColorDialog::setCustomColor(1, QColor(  0,   0,   0));
    QColorDialog::setCustomColor(2, QColor(255,   0,   0));
    QColorDialog::setCustomColor(3, QColor(  0, 255,   0));
    QColorDialog::setCustomColor(4, QColor(  0,   0, 255));
    QColorDialog::setCustomColor(5, QColor(255, 255,   0));
    QColorDialog::setCustomColor(6, QColor(  0, 255, 255));
    QColorDialog::setCustomColor(7, QColor(255,   0, 255));
    QColorDialog::setCustomColor(8, QColor( 51,  51, 102));
}

Application::~Application() {

}
void Application::initSheetStyle() {
    _ssht->init(&_param_list);
}
void Application::initTranslations() {
    _trsl->init(&_param_list);
}

bool Application::installHelpMenu()
{
    auto imw = IMainWindow::Instance();
    if (!imw) {
        return false;
    }
    _ssht->installToMenu(imw->getHelpMenu());
    _trsl->installToMenu(imw->getHelpMenu());
    return true;
}

void Application::SetSurfaceFormat()
{
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
#ifdef OSG_GL3_AVAILABLE
    format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setOption(QSurfaceFormat::DebugContext);
#else
    format.setVersion(2, 0);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setOption(QSurfaceFormat::DebugContext);
#endif
    format.setDepthBufferSize(24);
    //format.setAlphaBufferSize(8);
    format.setSamples(8);
    format.setStencilBufferSize(8);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    QSurfaceFormat::setDefaultFormat(format);
}

}
