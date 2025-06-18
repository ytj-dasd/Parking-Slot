add_executable(bamboo_test_dxfio main.cpp dxf_io.cpp)

target_link_libraries(bamboo_test_dxfio
    ${WELKIN_LIBDXFRW_LIB}
    ${WELKIN_QT_LIB}
    ${WELKIN_PCL_LIB}
    ${WELKIN_VTK_LIB}
    ${WELKIN_OPENCV_LIB}
    ${WELKIN_COMMON_LIB}
    ${WELKIN_OSG_LIB}
    ${WELKIN_UFACE_LIB}
)
