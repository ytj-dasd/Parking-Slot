include(./boost.pri)
#include(./flann.pri)
include(./vtk.pri)

win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = pcl-1.13.1
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = d
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_common$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_features$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_filters$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_io$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_io_ply$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_kdtree$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_keypoints$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_ml$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_octree$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_outofcore$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_people$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_recognition$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_registration$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_sample_consensus$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_search$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_segmentation$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_stereo$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_surface$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_tracking$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lpcl_visualization$$SUBFIX
}
unix{
    INCLUDEPATH += /usr/include/pcl-1.12

    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_common
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_features
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_filters
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_io
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_io_ply
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_kdtree
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_keypoints
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_ml
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_octree
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_outofcore
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_people
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_recognition
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_registration
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_sample_consensus
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_search
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_segmentation
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_stereo
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_surface
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_tracking
    LIBS +=-L/usr/lib/x86_64-linux-gnu -lpcl_visualization

}
