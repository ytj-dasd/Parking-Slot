#include "dxffile.h"
//
#include "dxflib/dl_dxf.h"
#include <uface/base/uconverter.h>
#include <uface/logger/ulogger.h>

namespace welkin::bamboo {
class DXFFilePrivate {
public:
    DXFFilePrivate() {

    }
    ~DXFFilePrivate() {
        this->close();
    }

    bool open(const std::string& filename, const DXFFile::LayerList& layers) {
        if (_dxf) {
            // 这里输出的是已经打开的文件名
            UWarn(QObject::tr("Already open: %1").arg(UTQ(_filename)));
            return true;
        }
        _dxf = new DL_Dxf();
        DL_Codes::version export_version = DL_Codes::AC1015;
//        DL_Codes::version export_version = DL_Codes::AC1027;
        _filename = filename;
        _dw = _dxf->out(_filename.c_str(), export_version);
        if (_dw == NULL) {
            UError(QObject::tr("Cannot open file dxf file for writing: %1").arg(UTQ(_filename)));
            delete _dxf;
            return false;
        }

        // 默认写头
        _dxf->writeHeader(*_dw);
        //
        _dw->sectionEnd();

        // 写配置表
        _dw->sectionTables();
        _dxf->writeVPort(*_dw);
        _dw->tableLinetypes(3);
        _dxf->writeLinetype(*_dw, DL_LinetypeData("BYBLOCK", "BYBLOCK", 0, 0, 0.0));
        _dxf->writeLinetype(*_dw, DL_LinetypeData("BYLAYER", "BYLAYER", 0, 0, 0.0));
        _dxf->writeLinetype(*_dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
        _dw->tableEnd();

        // 写默认层
        if (layers.empty()) {
            int numberOfLayers = 1;
            _dw->tableLayers(numberOfLayers);

            _dxf->writeLayer(*_dw,
                        DL_LayerData("0", 0),
                        DL_Attributes(
                        std::string(""),      // leave empty
                        DL_Codes::green,        // default color
                        1,                  // default width
                        "CONTINUOUS", 1.0));       // default line style

            _dw->tableEnd();
        } else { // 写入不同层
            int numberOfLayers = layers.size() + 1;
            _dw->tableLayers(numberOfLayers);
            _dxf->writeLayer(*_dw,
                        DL_LayerData("0", 0),
                        DL_Attributes(
                        std::string(""),      // leave empty
                        DL_Codes::green,        // default color
                        1,                  // default width
                        "CONTINUOUS", 1.0));       // default line style
            for (int i = 0; i < layers.size(); ++i) {
                _dxf->writeLayer(*_dw,
                        DL_LayerData(layers.at(i), 0),
                        DL_Attributes(
                        std::string(""),      // leave empty
                        DL_Codes::green,        // default color
                        1,                  // default width
                        "CONTINUOUS", 1.0));       // default line style
            }
            _dw->tableEnd();
        }

        _dw->tableStyle(1);
        _dxf->writeStyle(*_dw, DL_StyleData("standard", 0, 2.5, 1.0, 0.0, 0, 2.5, "txt", ""));
        _dw->tableEnd();

        _dxf->writeView(*_dw);
        _dxf->writeUcs(*_dw);

        _dw->tableAppid(1);
        _dxf->writeAppid(*_dw, "ACAD");
        _dw->tableEnd();
        _dxf->writeDimStyle(*_dw, 1, 1, 1, 1, 1);

        _dxf->writeBlockRecord(*_dw);
        // dxf->writeBlockRecord(*dw, "myblock1");
        // dxf->writeBlockRecord(*dw, "myblock2");
        _dw->tableEnd();
        _dw->sectionEnd();

        _dw->sectionBlocks();
        _dxf->writeBlock(*_dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
        _dxf->writeEndBlock(*_dw, "*Model_Space");
        _dxf->writeBlock(*_dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
        _dxf->writeEndBlock(*_dw, "*Paper_Space");
        _dxf->writeBlock(*_dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
        _dxf->writeEndBlock(*_dw, "*Paper_Space0");

        // dxf->writeBlock(*dw, DL_BlockData("myblock1", 0, 0.0, 0.0, 0.0));
        // // ...
        // // write block entities e.g. with dxf->writeLine(), ..
        // // ...
        // dxf->writeEndBlock(*dw, "myblock1");
    //
        // dxf->writeBlock(*dw, DL_BlockData("myblock2", 0, 0.0, 0.0, 0.0));
        // // ...
        // // write block entities e.g. with dxf->writeLine(), ..
        // // ...
        // dxf->writeEndBlock(*dw, "myblock2");
        _dw->sectionEnd();
        return true;
    }
    void writePoint3(const common::Point3d& pt, const std::string& layer_id) {
        _dxf->writePoint(*_dw,
            DL_PointData(pt.x, pt.y, pt.z),
            DL_Attributes(layer_id, 256, -1, "BYLAYER", 1.0));
    }
    void writePolyline2(const std::vector<common::Point2d>& points, 
            double default_z, bool closed, const std::string& layer_id) {
        std::size_t pt_num = points.size();
        _dw->sectionEntities();
        int flags = closed ? 1 : 0;
        _dxf->writePolyline(*_dw,
            DL_PolylineData(pt_num, 0, 0, flags),
            DL_Attributes(layer_id, 256, -1, "BYLAYER", 1.0));

        for (std::size_t i = 0; i < pt_num; ++i) {
            const auto& pt = points[i];
            _dxf->writeVertex(*_dw, DL_VertexData(pt.x, pt.y, default_z, 0.0));
        }
        _dxf->writePolylineEnd(*_dw);
        _dw->sectionEnd();
    }
    void writePolyline3(const std::vector<common::Point3d>& points, 
            bool closed, const std::string& layer_id) {
        std::size_t pt_num = points.size();
        _dw->sectionEntities();
        int flags = closed ? 1 : 0;
        _dxf->writePolyline(*_dw,
            DL_PolylineData(pt_num, 0, 0, flags),
            DL_Attributes(layer_id, 256, -1, "BYLAYER", 1.0));

        for (std::size_t i = 0; i < pt_num; ++i) {
            const auto& pt = points[i];
            _dxf->writeVertex(*_dw, DL_VertexData(pt.x, pt.y, pt.z, 0.0));
        }
        _dxf->writePolylineEnd(*_dw);
        _dw->sectionEnd();
    }
    void writeLayer(const common::Layerd& layer, const std::string& layer_id) {
        for (auto& point : layer.points) {
            this->writePoint3(point, layer_id);
        }
        for (auto& polyline : layer.polylines) {
            this->writePolyline3(polyline.points, false,layer_id);
        }
        for (auto& polygon : layer.polygons) {
            this->writePolyline3(polygon.points, true, layer_id);
        }
    }
    void writeImage(
            const std::string& filename, 
            int width, int height,
            const common::Point3d& insert_point, 
            const common::Point3d& uvec, 
            const common::Point3d& vvec,
            const std::string& layer_id) {
        DL_ImageData image_data = DL_ImageData(
            filename,
            insert_point.x, insert_point.y, insert_point.z,
            uvec.x, uvec.y, uvec.z,
            vvec.x, vvec.y, vvec.z,
            width, height,
            50, 50, 0
        );
        int handle = _dxf->writeImage(*_dw, image_data, 
            DL_Attributes(layer_id, 256, -1, "BYLAYER", 1.0));
        _dxf->writeImageDef(*_dw, handle, image_data);
    }
    void endWrite() {
        if (_dxf) {
            _dxf->writeObjects(*_dw);
            _dxf->writeObjectsEnd(*_dw);
        }
    }
    void close() {
        if (_dxf) {
            _dw->dxfEOF();
            _dw->close();
            delete _dw;
            delete _dxf;
            _dw = nullptr;
            _dxf = nullptr;
            _filename = std::string();
        }
    }

private:
    DL_Dxf* _dxf = nullptr;
    DL_WriterA* _dw = nullptr;
    std::string _filename;
    friend class DXFFile;
};

DXFFile::DXFFile() {
    if (!_ptr) {
        _ptr = new DXFFilePrivate();
    }
}
DXFFile::~DXFFile() {
    if (_ptr) {
        delete _ptr;
        _ptr = nullptr;
    }
}

bool DXFFile::open(const std::string& filename, const LayerList& layers) {
    return _ptr->open(filename, layers);
}
void DXFFile::close() {
    _ptr->endWrite();
    _ptr->close();
}
void DXFFile::writePoint3(const common::Point3d& pt, const std::string& layer_id) {
    _ptr->writePoint3(pt, layer_id);
}
void DXFFile::writePolyline2(const std::vector<common::Point2d>& points,
        double default_z, bool closed, const std::string& layer_id) {
    _ptr->writePolyline2(points, default_z, closed, layer_id);
}
void DXFFile::writePolyline3(const std::vector<common::Point3d>& points, 
        bool closed, const std::string& layer_id) {
    _ptr->writePolyline3(points, closed,layer_id);
}
void DXFFile::writeImage(
        const std::string& filename, 
        int width, int height,
        const common::Point3d& insert_point, 
        const common::Point3d& uvec, 
        const common::Point3d& vvec,
        const std::string& layer_id) {
    _ptr->writeImage(filename, width, height, insert_point, uvec, vvec, layer_id);
}
void DXFFile::writeLayer(const common::Layerd& layer, const std::string& layer_id) {
    _ptr->writeLayer(layer, layer_id);
}

}
