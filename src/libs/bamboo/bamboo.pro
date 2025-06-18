#-------------------------------------------------
#
# Project created by QtCreator 2015-08-12T07:01:57
#
#-------------------------------------------------

QT       += core gui widgets opengl
greaterThan(QT_MAJOR_VERSION, 5): QT += openglwidgets

TEMPLATE = lib

DEFINES += BAMBOO_LIB

TARGET = bamboo
include(../../depends/options.pri)
include(../../depends/base.pri)
include(../../depends/opencv.pri)
include(../../depends/pcl.pri)
include(../../depends/osg.pri)
include(../../depends/laszip.pri)
include(../../depends/dxflib.pri)
include(../../depends/libdxfrw.pri)
include(../../depends/common.pri)
include(../../depends/flann.pri)
include(../../depends/ceres.pri)
include(../../depends/sam.pri)

win32 {
    #翻译lupdate排除第三方库
    TR_EXCLUDE += $$BAMBOO_LIB_PATH/include
}
unix{
    #翻译lupdate排除第三方库
    TR_EXCLUDE += /welkin/panda/include/platform/* \
        /usr/include/*
}
## CONFIG += debug_and_release

INCLUDEPATH +=$$PWD/../

FORMS += \
    actions/model_param_dialog.ui

HEADERS += \
    actions/dom_param_dialog.h \
    actions/make_dom_action.h \
    actions/make_model_list_action.h \
    actions/model_param_dialog.h \
    application/application.h \
    base/converter.h \
    base/macro.h \
    base/osg_utils.h \
    base/string.h \
    base/types.h \
    cloud_viewer/cloud_viewer.h \
    combined_viewer/combined_command.h \
    combined_viewer/combined_shape_layer.h \
    combined_viewer/combined_viewer.h \
    file_system/building_folder.h \
    file_system/origin_data_folder.h \
    file_system/project.h \
    file_system/raw_data_folder.h \
    file_system/terrain_folder.h \
    file_system/types.h \
    image_viewer/dialog/image_pair_dialog.h \
    image_viewer/dialog/rect_image_dialog.h \
    image_viewer/dialog/select_line_dialog.h \
    image_viewer/dialog/select_pillar_dialog.h \
    image_viewer/dialog/select_polygon_dialog.h \
    image_viewer/dialog/select_rect_dialog.h \
    image_viewer/item/graphics_road_arrow_item.h \
    image_viewer/item/graphics_section_item.h \
    image_viewer/item/graphics_text_polygon_item.h \
    image_viewer/layer/draw_shape_layer.h \
    image_viewer/layer/id_mark_shape_layer.h \
    image_viewer/layer/section_item_layer.h \
    image_viewer/layer/text_polygon_item_layer.h \
    io/dx_data.h \
    io/dx_iface.h \
    io/dxf_io.h \
    io/dxf_reader.h \
    io/dxffile.h \
    io/las_point.h \
    io/las_reader.h \
    io/las_writer.h \
    osg_viewer/drive_manipulator.h \
    osg_viewer/layer/osg_cloud_layer.h \
    osg_viewer/layer/osg_geode_layer.h \
    osg_viewer/layer/osg_group_layer.h \
    osg_viewer/layer/osg_model_layer.h \
    osg_viewer/layer/osg_mono_cloud_layer.h \
    osg_viewer/layer/osg_shape_layer.h \
    osg_viewer/osg_layer.h \
    osg_viewer/osg_render.h \
    osg_viewer/osg_viewer.h \
    plugin/iplugin.h \
    plugin/tool_plugin.h \
    plugin/tool_plugin.h \
    utils/DBSCAN_kdtree.h \
    utils/DBSCAN_precomp.h \
    utils/DBSCAN_simple.h \
    utils/canny.h \
    utils/clipper.hpp \
    utils/cloud_algorithm.h \
    utils/curb.h \
    utils/geometry.h \
    utils/ground_extraction.h \
    utils/image_algorithm.h \
    utils/lane.h \
    utils/park_lot.h \
    utils/piecewise_line_fit.h \
    utils/plicp.h \
    utils/pole.h \
    utils/road_arrow.h \
    utils/sam_predict.h \
    widget/bproject_action.h \
    widget/bproject_widget.h \
    widget/colormap_combobox.h \
    widget/d2d3_viewer.h

SOURCES += \
    actions/dom_param_dialog.cpp \
    actions/make_dom_action.cpp \
    actions/make_model_list_action.cpp \
    actions/model_param_dialog.cpp \
    application/application.cpp \
    base/converter.cpp \
    base/osg_utils.cpp \
    base/string.cpp \
    cloud_viewer/cloud_viewer.cpp \
    combined_viewer/combined_command.cpp \
    combined_viewer/combined_shape_layer.cpp \
    combined_viewer/combined_viewer.cpp \
    file_system/building_folder.cpp \
    file_system/impl/make_cloud_model.cpp \
    file_system/origin_data_folder.cpp \
    file_system/project.cpp \
    file_system/raw_data_folder.cpp \
    file_system/terrain_folder.cpp \
    image_viewer/dialog/image_pair_dialog.cpp \
    image_viewer/dialog/rect_image_dialog.cpp \
    image_viewer/dialog/select_line_dialog.cpp \
    image_viewer/dialog/select_pillar_dialog.cpp \
    image_viewer/dialog/select_polygon_dialog.cpp \
    image_viewer/dialog/select_rect_dialog.cpp \
    image_viewer/item/graphics_road_arrow_item.cpp \
    image_viewer/item/graphics_section_item.cpp \
    image_viewer/item/graphics_text_polygon_item.cpp \
    image_viewer/layer/draw_shape_layer.cpp \
    image_viewer/layer/id_mark_shape_layer.cpp \
    image_viewer/layer/section_item_layer.cpp \
    image_viewer/layer/text_polygon_item_layer.cpp \
    io/dx_iface.cpp \
    io/dxf_io.cpp \
    io/dxf_reader.cpp \
    io/dxffile.cpp \
    io/las_reader.cpp \
    io/las_writer.cpp \
    osg_viewer/drive_manipulator.cpp \
    osg_viewer/layer/osg_cloud_layer.cpp \
    osg_viewer/layer/osg_geode_layer.cpp \
    osg_viewer/layer/osg_group_layer.cpp \
    osg_viewer/layer/osg_model_layer.cpp \
    osg_viewer/layer/osg_mono_cloud_layer.cpp \
    osg_viewer/layer/osg_shape_layer.cpp \
    osg_viewer/osg_layer.cpp \
    osg_viewer/osg_render.cpp \
    osg_viewer/osg_viewer.cpp \
    plugin/tool_plugin.cpp \
    utils/canny.cpp \
    utils/clipper.cpp \
    utils/cloud_algorithm.cpp \
    utils/curb.cpp \
    utils/geometry.cpp \
    utils/ground_extraction.cpp \
    utils/image_algorithm.cpp \
    utils/lane.cpp \
    utils/park_lot.cpp \
    utils/piecewise_line_fit.cpp \
    utils/plicp.cpp \
    utils/pole.cpp \
    utils/road_arrow.cpp \
    utils/sam_predict.cpp \
    widget/bproject_action.cpp \
    widget/bproject_widget.cpp \
    widget/colormap_combobox.cpp \
    widget/d2d3_viewer.cpp

RESOURCES += \
    resource/bamboo.qrc

