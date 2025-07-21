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
    actions/auto_detection_dialog.ui \
    actions/detect_corner_dialog.ui \
    actions/export_layer_as_dxf_dialog.ui \
    actions/generate_slot_dialog.ui \
    actions/import_layers_dialog.ui \
    actions/import_detection_dialog.ui \
    actions/match_template_dialog.ui


HEADERS +=  \
    actions/auto_detection_dialog.h \
    actions/detect_corner_action.h \
    actions/detect_corner_dialog.h \
    actions/export_layer_as_dxf_action.h \
    actions/export_layer_as_dxf_dialog.h \
    actions/generate_slot_action.h \
    actions/generate_slot_dialog.h \
    actions/import_layers_action.h \
    actions/import_layers_dialog.h \
    actions/import_detection_action.h \
    actions/import_detection_dialog.h \
    actions/match_template_action.h \
    actions/match_template_dialog.h \
    main_viewer.h \
    mainwindow.h \
    parkline.h \
    parkspace.h \
    parkspacegroup.h \
    project_widget.h


SOURCES +=  \
    actions/auto_detection_dialog.cpp \
    actions/detect_corner_action.cpp \
    actions/detect_corner_dialog.cpp \
    actions/export_layer_as_dxf_action.cpp \
    actions/export_layer_as_dxf_dialog.cpp \
    actions/generate_slot_action.cpp \
    actions/generate_slot_dialog.cpp \
    actions/import_layers_action.cpp \
    actions/import_layers_dialog.cpp \
    actions/import_detection_action.cpp \
    actions/import_detection_dialog.cpp \
    actions/match_template_action.cpp \
    actions/match_template_dialog.cpp \
    main.cpp \
    main_viewer.cpp \
    mainwindow.cpp \
    parkline.cpp \
    parkspace.cpp \
    parkspacegroup.cpp \
    project_widget.cpp
