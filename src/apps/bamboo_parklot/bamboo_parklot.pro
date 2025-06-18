#-------------------------------------------------
#
# Project created by QtCreator 2015-08-12T20:37:36
#
#-------------------------------------------------

QT       += core gui widgets opengl
greaterThan(QT_MAJOR_VERSION, 5): QT += openglwidgets

TEMPLATE = app

TARGET = bamboo_parklot
include(../../depends/options.pri)
include(../../libs/libs.pri)
include(../../depends/opencv.pri)
include(../../depends/pcl.pri)
include(../../depends/osg.pri)
include(../../depends/flann.pri)
include(../../depends/ceres.pri)

RC_ICONS = bamboo_parklot.ico

FORMS +=  \
    actions/export_layer_as_dxf_dialog.ui \
    actions/import_layers_dialog.ui


HEADERS +=  \
    actions/export_layer_as_dxf_action.h \
    actions/export_layer_as_dxf_dialog.h \
    actions/import_layers_action.h \
    actions/import_layers_dialog.h \
    main_viewer.h \
    mainwindow.h \
    parkline.h \
    parkspace.h \
    parkspacegroup.h \
    project_widget.h


SOURCES +=  \
    actions/export_layer_as_dxf_action.cpp \
    actions/export_layer_as_dxf_dialog.cpp \
    actions/import_layers_action.cpp \
    actions/import_layers_dialog.cpp \
    main.cpp \
    main_viewer.cpp \
    mainwindow.cpp \
    parkline.cpp \
    parkspace.cpp \
    parkspacegroup.cpp \
    project_widget.cpp
