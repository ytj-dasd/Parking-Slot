#include "las_reader.h"
#include <common/base/string.h>
#include <laszip/laszip_api.h>
#include "bamboo/base/string.h"

namespace welkin::bamboo {
uint8_t Color8Bits(uint16_t color16Bit) {
	return (color16Bit < 256) ? color16Bit 
        : static_cast<uint8_t>((color16Bit / 65535.0) * 255.0);
};
class LasReaderPrivate {
public:
    LasReaderPrivate(LasReader* pthis) : _pthis(pthis) {
        laszip_create(&_laszip_reader);
    }
    virtual ~LasReaderPrivate() {
        laszip_destroy(_laszip_reader);
    }
    bool open(const std::string filename) {
        laszip_BOOL is_compressed = common::String(filename).isEndWith(".laz");
        laszip_create(&_laszip_reader);
        // laszip中统一采用utf8字符
        laszip_open_reader(_laszip_reader, StringToUTF8(filename).c_str(), &is_compressed);
        laszip_get_header_pointer(_laszip_reader, &_laszip_header);
        laszip_get_point_pointer(_laszip_reader, &_point_read);
        return true;
    }
    void close() {
        laszip_close_reader(_laszip_reader);
    }
    size_t pointCount() const {
        size_t point_count = (_laszip_header->number_of_point_records ? 
		    _laszip_header->number_of_point_records : 
            _laszip_header->extended_number_of_point_records);
        return point_count;
    }
    common::Boxd getBBox() const {
        return common::Boxd(
            _laszip_header->min_x, _laszip_header->max_x,
            _laszip_header->min_y, _laszip_header->max_y,
            _laszip_header->min_z, _laszip_header->max_z
        );
    }
    common::Point3d getOffset() const {
        return common::Point3d(
            _laszip_header->x_offset,
            _laszip_header->y_offset,
            _laszip_header->z_offset
        );
    }
    void readPoint(LasPoint& point) {
        laszip_read_point(_laszip_reader);
        laszip_get_coordinates(_laszip_reader, point.coordinates);
        auto& rgb = _point_read->rgb;
        point.rgb[0] = Color8Bits(rgb[0]);
        point.rgb[1] = Color8Bits(rgb[1]);
        point.rgb[2] = Color8Bits(rgb[2]);
    }
private:
    friend class LasReader;
    LasReader* _pthis = nullptr;

    laszip_POINTER _laszip_reader;
	laszip_header* _laszip_header;
    laszip_point* _point_read;
};

LasReader::LasReader() {
    if (!_ptr) {
        _ptr = new LasReaderPrivate(this);
    }
}
LasReader::~LasReader() {
    if (_ptr) {
        delete _ptr;
        _ptr = nullptr;
    }
}
bool LasReader::open(const std::string& filename) {
    return _ptr->open(filename);
}
void LasReader::close() {
    _ptr->close();
}
size_t LasReader::pointCount() const {
    return _ptr->pointCount();
}
common::Boxd LasReader::getBBox() const {
    return _ptr->getBBox();
}
common::Point3d LasReader::getOffset() const {
    return _ptr->getOffset();
}
void LasReader::readPoint(LasPoint& point) {
    _ptr->readPoint(point);
}
}