#include "dxf_reader.h"
#include "dxflib/dl_dxf.h"
#include "dxflib/dl_creationadapter.h"

namespace welkin::bamboo {
class DxfReaderPrivate  : public DL_CreationAdapter {
public:
   DxfReaderPrivate(DxfReader* pthis) : _pthis(pthis) {}
	virtual ~DxfReaderPrivate() {}

   bool open(const std::string& filepath) {
       DL_Dxf dxf;
       if (!dxf.in(filepath, this)) { // if file open failed
   	    AERROR << filepath << " could not be opened!";
           return false;
       }
       return this->parseDxf();
   }
   void close() {
   	m_vecDxfPolylines.clear();
   	m_vecDxfVertexs.clear();
   	_polygons.clear();
   	_polylines.clear();
   }
protected:
   /// private
   void addPolyline(const DL_PolylineData& data) {
       m_vecDxfPolylines.push_back(std::pair<DL_PolylineData, DL_Attributes>(data, attributes));
   }
   void addVertex(const DL_VertexData& data) {
   	m_vecDxfVertexs.push_back(std::pair<DL_VertexData, DL_Attributes>(data, attributes));
   }
   bool parseDxf() {
   	_polygons.clear();
   	_polylines.clear();
       std::vector<std::pair<DL_PolylineData, DL_Attributes>>::iterator iter;
   	int index = 0;//当前线段的第一个点在m_vecDxfVertexs中的索引
   	for (iter = m_vecDxfPolylines.begin(); iter < m_vecDxfPolylines.end(); iter++) {
   		DL_PolylineData &data = (*iter).first;
   		int countVertex = data.number;//该线段含有几个点
           common::Point3dList points;
   		for (int subIndex = 0; subIndex < countVertex; subIndex++) {
   			//取折线的第二个点
   			DL_VertexData curVertex = m_vecDxfVertexs[index + subIndex].first;
   			if (curVertex.bulge == 0) {
                   points.push_back(common::Point3d(curVertex.x, curVertex.y, curVertex.z));
   			} else { //圆弧
   				// TODO:
                   AWARN << "圆弧暂时无法处理";
   			}
   		}
   		if (data.flags == 1) {
               common::Polygon3d polygon;
               polygon.points = points;
   			_polygons.push_back(polygon);
   		} else {
               common::Polyline3d polyline;
               polyline.points = points;
               _polylines.push_back(polyline);
           }
   		index = index + data.number;
   	}
   	return true;
   }

private:
   friend class DxfReader;
	DxfReader* _pthis = nullptr;

	std::vector<std::pair<DL_PolylineData, DL_Attributes>>  m_vecDxfPolylines;
	std::vector<std::pair<DL_VertexData, DL_Attributes>>  m_vecDxfVertexs;

   std::vector<common::Polygon3d> _polygons;
   std::vector<common::Polyline3d> _polylines;
};

DxfReader::DxfReader() {
	if (!_ptr) {
		_ptr = new DxfReaderPrivate(this);
	}
}
DxfReader::~DxfReader() {
	if (_ptr) {
		delete _ptr;
		_ptr = nullptr;
	}
}

bool DxfReader::open(const std::string& filepath) {
    return _ptr->open(filepath);
}
void DxfReader::close() {
	_ptr->close();
}
const std::vector<common::Polygon3d>& DxfReader::getPolygonVec() const {
	return _ptr->_polygons;
}
const std::vector<common::Polyline3d>& DxfReader::getPolylineVec() const {
	return _ptr->_polylines;
}
}
