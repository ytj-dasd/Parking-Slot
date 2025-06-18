win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = opencv-4.7
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = d
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_calib3d470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_core470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_dnn470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_features2d470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_flann470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_gapi470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_highgui470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_imgcodecs470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_imgproc470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_ml470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_objdetect470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_photo470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_stitching470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_video470$$SUBFIX
    LIBS  +=-L$$BAMBOO_LIB_PATH -lopencv_videoio470$$SUBFIX
}
unix{
    INCLUDEPATH += /usr/include/opencv4

    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_calib3d
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_core
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_dnn
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_features2d
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_flann
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_highgui
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_imgcodecs
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_imgproc
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_ml
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_objdetect
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_photo
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_stitching
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_video
    LIBS  +=-L/usr/lib/x86_64-linux-gnu -lopencv_videoio
}
