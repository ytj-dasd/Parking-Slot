#include "dxf_io.h"

using namespace welkin;
using namespace welkin::bamboo;
int main(int argc, char** argv) {
    // if (false) {
    //     DxfOutput dxf_output;
    //     if (!dxf_output.open("D:/test/test.dxf")) {
    //         std::cout << "Open failed!" << std::endl;
    //         return -1;
    //     }

    //     // 插入图片
    //     std::vector<std::string> layer_names = {"test"};
    //     dxf_output.writeLayerIds(layer_names);

    //     std::string image_path = "001_Floor_001.jpg";
    //     int width = 1964; int height = 972;
    //     common::Point3d origin(40, 40, 0);
    //     common::Point3d uvec(0.05, 0.0866025, 0.0);
    //     common::Point3d vvec(0.0866025, -0.05, 0.0);
    //     dxf_output.writeImage(image_path, width, height, origin, uvec, vvec, "test");
    //     // 插入点
    // //    dxf_output.writePoint3(common::Point3d(100, 100, 0), "test");
    //     // 插入线
    // //    common::Line3d line(common::Point3d(0, 0, 0), common::Point3d(-100.9, 100.9, 0));
    // //    dxf_output.writeLine3(line, "test");

    //     // 插入多线
    // //    common::Point3dList points;
    // //    points.push_back(common::Point3d(0, 0, 0));
    // //    points.push_back(common::Point3d(-100.9, 100.9, 0));
    // //    points.push_back(common::Point3d(50.9, 30.9, 0));
    // //    dxf_output.writePolyline3(points, true, "test");
    // //    std::cout << "write polyline" << std::endl;
    //     if (!dxf_output.close()) {
    //         std::cout << "Open failed!" << std::endl;
    //         return -1;
    //     }
    //     std::cout << "Save successfully!" << std::endl;
    // }
    {
        std::string dxf_filename = argv[1];
        bamboo::DxfInput dxf_input;
        if (!dxf_input.open(dxf_filename)) {
            std::cout << "Open dxf file failed!" << std::endl;
            return -1;
        }
        dxf_input.close();
        std::cout << "Point: " << dxf_input.getPointVec().size() << std::endl;
        std::cout << "Line: " << dxf_input.getLineVec().size() << std::endl;
        std::cout << "Polyline: " << dxf_input.getPolylineVec().size() << std::endl;
        std::cout << "Polygon: " << dxf_input.getPolygonVec().size() << std::endl;
        std::cout << "Arc: " << dxf_input.getArcVec().size() << std::endl;
        std::cout << "Circle: " << dxf_input.getCircleVec().size() << std::endl;
    }
    return 0;
}
