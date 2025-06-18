#-------------------------------------------------
#
# Project created by QtCreator 2015-08-12T07:01:57
#
#-------------------------------------------------

QT       += core gui widgets opengl openglwidgets

TEMPLATE = lib

DEFINES += BAMBOO_TOOL_CONVERTOR_LIB
CONFIG += c++14
CONFIG += plugin

include(../../libs/libs.pri)

INCLUDEPATH +=$$PWD/../

SOURCES += \
    cloud_convertor_widget.cpp \
    plugin.cpp

HEADERS += \
    cloud_convertor_widget.h \
    plugin.h


Debug {
TARGET = bamboo_tool_convertor_d
DLLDESTDIR += ../../bin/plugin/tools
}
Release{
TARGET = bamboo_tool_convertor
DLLDESTDIR +=../../bin/plugin/tools
}

FORMS += \
    cloud_convertor_widget.ui

