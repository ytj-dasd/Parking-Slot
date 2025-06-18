#-------------------------------------------------
#
# Project created by QtCreator 2015-08-12T20:37:36
#
#-------------------------------------------------

QT       += core gui widgets opengl
greaterThan(QT_MAJOR_VERSION, 5): QT += openglwidgets

TEMPLATE = app

TARGET = dxfio
include(../../depends/options.pri)
include(../../libs/libs.pri)
include(../../depends/opencv.pri)
include(../../depends/pcl.pri)
include(../../depends/osg.pri)
include(../../depends/libdxfrw.pri)

SOURCES += main.cpp
