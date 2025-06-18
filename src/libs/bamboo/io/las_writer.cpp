#include "las_writer.h"
#include <laszip/laszip_api.h>
#include <uface/uface.hpp>
#include "bamboo/base/string.h"
namespace welkin::bamboo {
class LasWriterPrivate {
public:
    LasWriterPrivate(LasWriter* pthis) : _pthis(pthis) {}
    virtual ~LasWriterPrivate() {}
    bool open(const std::string& filename, const common::Boxd& bbox, double scale) {
        if (laszip_create(&_laszip_writer)) {
            UError(QObject::tr("Error create laszip writer"));
            return false;
        }
        if (laszip_get_header_pointer(_laszip_writer, &_laszip_header)) {
            UError(QObject::tr("Error get header pointer from laszip writer"));
            return false;
        }
        _filename = filename;
        strcpy(_laszip_header->generating_software, "gtop");
        _laszip_header->version_major = 1;
		_laszip_header->version_minor = 2;
		_laszip_header->header_size = 227;
		_laszip_header->offset_to_point_data = 227;
		_laszip_header->point_data_format = 2;
		_laszip_header->point_data_record_length = 26;

		_laszip_header->min_x = bbox.getMinX();
		_laszip_header->min_y = bbox.getMinY();
		_laszip_header->min_z = bbox.getMinZ();
		_laszip_header->max_x = bbox.getMaxX();
		_laszip_header->max_y = bbox.getMaxY();
		_laszip_header->max_z = bbox.getMaxZ();
        _laszip_header->x_offset = _laszip_header->min_x;
        _laszip_header->y_offset = _laszip_header->min_y;
        _laszip_header->z_offset = _laszip_header->min_z;
        // 保证毫米级别精度
        _laszip_header->x_scale_factor = scale;
        _laszip_header->y_scale_factor = scale;
        _laszip_header->z_scale_factor = scale; 
		_laszip_header->number_of_point_records = 0;
        
        laszip_BOOL compress = common::String(_filename).isEndWith(".laz") ? true : false;
        if(compress){
			laszip_BOOL request_writer = 1;
			laszip_request_compatibility_mode(_laszip_writer, request_writer);
		}
        laszip_open_writer(_laszip_writer, StringToUTF8(_filename).c_str(), compress);
        laszip_get_point_pointer(_laszip_writer, &_laszip_point);
        return true;
    }
    size_t pointCount() const {
        size_t point_count = (_laszip_header->number_of_point_records ? 
		    _laszip_header->number_of_point_records : 
            _laszip_header->extended_number_of_point_records);
        return point_count;
    }
    void writePoint(const LasPoint& point) {
        laszip_set_coordinates(_laszip_writer, point.coordinates);
        _laszip_point->rgb[0] = point.rgb[0] * 256;
        _laszip_point->rgb[1] = point.rgb[1] * 256;
        _laszip_point->rgb[2] = point.rgb[2] * 256;
        laszip_set_point(_laszip_writer, _laszip_point);
	    laszip_write_point(_laszip_writer);
        ++(_laszip_header->number_of_point_records);
    }
    void close() {
        if (_laszip_writer) {
            auto point_count = _laszip_header->number_of_point_records;
            UInfo(QObject::tr("las wirter point count: %1").arg(point_count));
            laszip_close_writer(_laszip_writer);
			laszip_destroy(_laszip_writer);
            _laszip_writer = nullptr;
            _laszip_point = nullptr;
            // 修改点数量
            std::fstream *stream = new std::fstream(
                _filename, ios::out | ios::binary | ios::in );
			stream->seekp(107);
			stream->write(reinterpret_cast<const char*>(&point_count), 4);
			stream->close();
			delete stream;
        }
    }
private:
    friend class LasWriter;
    LasWriter* _pthis = nullptr;

    std::string _filename = std::string();
    laszip_POINTER _laszip_writer = nullptr;
    laszip_point* _laszip_point = nullptr;
    laszip_header* _laszip_header = nullptr;
};

LasWriter::LasWriter() {
    if (!_ptr) {
        _ptr = new LasWriterPrivate(this);
    }
}
LasWriter::~LasWriter() {
    if (_ptr) {
        delete _ptr;
        _ptr = nullptr;
    }
}

bool LasWriter::open(const std::string& filename, const common::Boxd& bbox, double scale) {
    return _ptr->open(filename, bbox, scale);
}
size_t LasWriter::pointCount() const {
    return _ptr->pointCount();
}
void LasWriter::writePoint(const LasPoint& point) {
    _ptr->writePoint(point);
}
void LasWriter::close() {
    _ptr->close();
}
}
