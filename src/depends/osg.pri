win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = osg-3.6.5
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = d
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS +=-L$$BAMBOO_LIB_PATH -lOpenThreads$$SUBFIX   
    LIBS +=-L$$BAMBOO_LIB_PATH -losgGA$$SUBFIX            
    LIBS +=-L$$BAMBOO_LIB_PATH -losgShadow$$SUBFIX  
    LIBS +=-L$$BAMBOO_LIB_PATH -losgUtil$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -losg$$SUBFIX           
    LIBS +=-L$$BAMBOO_LIB_PATH -losgManipulator$$SUBFIX   
    LIBS +=-L$$BAMBOO_LIB_PATH -losgSim$$SUBFIX      
    LIBS +=-L$$BAMBOO_LIB_PATH -losgViewer$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -losgAnimation$$SUBFIX  
    LIBS +=-L$$BAMBOO_LIB_PATH -losgParticle$$SUBFIX      
    LIBS +=-L$$BAMBOO_LIB_PATH -losgTerrain$$SUBFIX  
    LIBS +=-L$$BAMBOO_LIB_PATH -losgVolume$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -losgDB$$SUBFIX         
    LIBS +=-L$$BAMBOO_LIB_PATH -losgPresentation$$SUBFIX  
    LIBS +=-L$$BAMBOO_LIB_PATH -losgText$$SUBFIX     
    LIBS +=-L$$BAMBOO_LIB_PATH -losgWidget$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -losgFX$$SUBFIX         
    LIBS +=-L$$BAMBOO_LIB_PATH -losgQOpenGL$$SUBFIX       
    LIBS +=-L$$BAMBOO_LIB_PATH -losgUI$$SUBFIX
}
unix {
    INCLUDEPATH += /usr/include

    LIBS +=-L/usr/libx86_64-linux-gnu -lOpenThreads
    LIBS +=-L/usr/libx86_64-linux-gnu -losgGA
    LIBS +=-L/usr/libx86_64-linux-gnu -losgShadow
    LIBS +=-L/usr/libx86_64-linux-gnu -losgUtil
    LIBS +=-L/usr/libx86_64-linux-gnu -losg
    LIBS +=-L/usr/libx86_64-linux-gnu -losgManipulator
    LIBS +=-L/usr/libx86_64-linux-gnu -losgSim
    LIBS +=-L/usr/libx86_64-linux-gnu -losgViewer
    LIBS +=-L/usr/libx86_64-linux-gnu -losgAnimation
    LIBS +=-L/usr/libx86_64-linux-gnu -losgParticle
    LIBS +=-L/usr/libx86_64-linux-gnu -losgTerrain
    LIBS +=-L/usr/libx86_64-linux-gnu -losgVolume
    LIBS +=-L/usr/libx86_64-linux-gnu -losgDB
    LIBS +=-L/usr/libx86_64-linux-gnu -losgPresentation
    LIBS +=-L/usr/libx86_64-linux-gnu -losgText
    LIBS +=-L/usr/libx86_64-linux-gnu -losgWidget
    LIBS +=-L/usr/libx86_64-linux-gnu -losgFX
    LIBS +=-L/usr/libx86_64-linux-gnu -losgUI

}
