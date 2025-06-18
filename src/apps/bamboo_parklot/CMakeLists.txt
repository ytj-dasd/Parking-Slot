file(GLOB_RECURSE SOURCE_FILE_LIST *.cpp) 
add_executable(bamboo_parklot
    ${SOURCE_FILE_LIST}
)

target_link_libraries(bamboo_parklot
    ${WELKIN_QT_LIB}
    ${WELKIN_PCL_LIB}
    ${WELKIN_VTK_LIB}
    ${WELKIN_OPENCV_LIB}
    ${WELKIN_COMMON_LIB}
    ${WELKIN_UFACE_LIB}
    ${BAMBOO_LIB}
)
