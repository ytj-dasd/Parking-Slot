# WELKIN_SET_LIB_NAME(welkin_bamboo)
WELKIN_SET_LIB_NAME(bamboo)

# 增加资源文件
SET(${WELKIN_LIB_NAME_UPPER}_SRC ./resource/bamboo.qrc)
WELKIN_ADD_LIBRARY(
    ./actions
    ./base
    ./io
    ./cloud_viewer
    ./file_system
    ./file_system/impl
    ./image_viewer/item
    ./image_viewer/layer
    ./image_viewer/dialog
    ./osg_viewer
    ./osg_viewer/layer
    ./combined_viewer
    ./plugin
    ./widget
    ./utils
    ./application
)

WELKIN_LINK_LIBRARIES(
    ${WELKIN_QT_LIB}
    ${WELKIN_PCL_LIB}
    ${WELKIN_VTK_LIB}
    ${WELKIN_OSG_LIB}
    ${WELKIN_DXF_LIB}
    ${WELKIN_OPENCV_LIB}
    ${WELKIN_COMMON_LIB}
    ${WELKIN_UFACE_LIB}
    ${WELKIN_CERES_LIB}
    ${WELKIN_LIBDXFRW_LIB}
    ${WELKIN_SAM_LIB}
)
