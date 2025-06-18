#include "dxf_io.h"
#include <libdxfrw/drw_interface.h>
#include <libdxfrw/libdxfrw.h>

namespace welkin::bamboo {
/// DxfInputPrivate
class DxfInputPrivate : public DRW_Interface {
public:
    DxfInputPrivate(DxfInput* pthis) : _pthis(pthis) {}
    virtual ~DxfInputPrivate() {}
    bool open(const std::string& filepath) {
        _dxfR = new dxfRW(filepath.c_str());
        _points.clear();
        _lines.clear();
        _polygons.clear();
        _polylines.clear();
        _circles.clear();
        _arcs.clear();
        bool success = _dxfR->read(this, false);
        delete _dxfR;
        return success;
    }
    bool close() {return true;}

protected:
    /** Called when header is parsed.  */
    void addHeader(const DRW_Header* data) override {}

    /** Called for every line Type.  */
    void addLType(const DRW_LType& data) override {}
    /** Called for every layer. */
    void addLayer(const DRW_Layer& data) override {}
    /** Called for every dim style. */
    void addDimStyle(const DRW_Dimstyle& data) override {}
    /** Called for every VPORT table. */
    void addVport(const DRW_Vport& data) override {}
    /** Called for every text style. */
    void addTextStyle(const DRW_Textstyle& data) override {}
    /** Called for every AppId entry. */
    void addAppId(const DRW_AppId& data) override {}

    /**
     * Called for every block. Note: all entities added after this
     * command go into this block until endBlock() is called.
     *
     * @see endBlock()
     */
    void addBlock(const DRW_Block& data) override {}

    /**
     * In DWG called when the following entities corresponding to a
     * block different from the current. Note: all entities added after this
     * command go into this block until setBlock() is called already.
     *
     * int handle are the value of DRW_Block::handleBlock added with addBlock()
     */
    void setBlock(const int handle) override {}

    /** Called to end the current block */
    void endBlock() override {}

    /** Called for every point */
    void addPoint(const DRW_Point& data) override {
        const auto& x = data.basePoint.x;
        const auto& y = data.basePoint.y;
        const auto& z = data.basePoint.z;
        _points.push_back(common::Point3d(x, y, z));
    }

    /** Called for every line */
    void addLine(const DRW_Line& data) override {
        common::Point3d begin_point(data.basePoint.x, data.basePoint.y, data.basePoint.z);
        common::Point3d end_point(data.secPoint.x, data.secPoint.y, data.secPoint.z);
        _lines.push_back(common::Line3d(begin_point, end_point));
    }

    /** Called for every ray */
    void addRay(const DRW_Ray& data) override {}

    /** Called for every xline */
    void addXline(const DRW_Xline& data) override {}

    /** Called for every arc */
    void addArc(const DRW_Arc& data) override {
        common::Arcd arc;
        arc.cnt_pt = common::Point2d(data.basePoint.x, data.basePoint.y);
        arc.range_radius = common::Ranged(0.0, data.radious);
        if (data.isccw) { // 顺时针
            arc.range_angle = common::Ranged(data.staangle, data.endangle);
        } else { // 逆时针
            arc.range_angle = common::Ranged(data.endangle, data.staangle);
        }
        _arcs.push_back(arc);
    }

    /** Called for every circle */
    void addCircle(const DRW_Circle& data) override {
        common::Circled circle;
        circle.cnt_pt = common::Point2d(data.basePoint.x, data.basePoint.y);
        circle.range_radius = common::Ranged(0.0, data.radious);
        _circles.push_back(circle);
    }

    /** Called for every ellipse */
    void addEllipse(const DRW_Ellipse& data) override {}

    /** Called for every lwpolyline */
    void addLWPolyline(const DRW_LWPolyline& data) override {
        common::Point3dList points;
        for (auto& vert : data.vertlist) {
            const auto& x = vert->x;
            const auto& y = vert->y;
            const auto& z = data.elevation;
            points.push_back(common::Point3d(x, y, z));
        }
        if (data.flags & 1) { // 闭合
            common::Polygon3d polygon;
            polygon.points = points;
            _polygons.push_back(polygon);
        } else {
            common::Polyline3d polyline;
            polyline.points = points;
            _polylines.push_back(polyline);
        }
    }

    /** Called for every polyline start */
    void addPolyline(const DRW_Polyline& data) override {
        common::Point3dList points;
        for (auto& vert : data.vertlist) {
            const auto& x = vert->basePoint.x;
            const auto& y = vert->basePoint.y;
            const auto& z = vert->basePoint.z;
            points.push_back(common::Point3d(x, y, z));
        }
        if (data.flags & 1) { // 闭合
            common::Polygon3d polygon;
            polygon.points = points;
            _polygons.push_back(polygon);
        } else {
            common::Polyline3d polyline;
            polyline.points = points;
            _polylines.push_back(polyline);
        }
    }

    /** Called for every spline */
    void addSpline(const DRW_Spline* data) override {}
	
	/** Called for every spline knot value */
    void addKnot(const DRW_Entity& data) override {}

    /** Called for every insert. */
    void addInsert(const DRW_Insert& data) override {}
    
    /** Called for every trace start */
    void addTrace(const DRW_Trace& data) override {}
    
    /** Called for every 3dface start */
    void add3dFace(const DRW_3Dface& data) override {}

    /** Called for every solid start */
    void addSolid(const DRW_Solid& data) override {}


    /** Called for every Multi Text entity. */
    void addMText(const DRW_MText& data) override {}

    /** Called for every Text entity. */
    void addText(const DRW_Text& data) override {}

    /**
     * Called for every aligned dimension entity. 
     */
    void addDimAlign(const DRW_DimAligned *data) override {}
    /**
     * Called for every linear or rotated dimension entity. 
     */
    void addDimLinear(const DRW_DimLinear *data) override {}

	/**
     * Called for every radial dimension entity. 
     */
    void addDimRadial(const DRW_DimRadial *data) override {}

	/**
     * Called for every diametric dimension entity. 
     */
    void addDimDiametric(const DRW_DimDiametric *data) override {}

	/**
     * Called for every angular dimension (2 lines version) entity. 
     */
    void addDimAngular(const DRW_DimAngular *data) override {}

	/**
     * Called for every angular dimension (3 points version) entity. 
     */
    void addDimAngular3P(const DRW_DimAngular3p *data) override {}
	
    /**
     * Called for every ordinate dimension entity. 
     */
    void addDimOrdinate(const DRW_DimOrdinate *data) override {}
    
    /** 
	 * Called for every leader start. 
	 */
    void addLeader(const DRW_Leader *data) override {}
	
	/** 
	 * Called for every hatch entity. 
	 */
    void addHatch(const DRW_Hatch *data) override {}
	
    /**
     * Called for every viewport entity.
     */
    void addViewport(const DRW_Viewport& data) override {}

    /**
	 * Called for every image entity. 
	 */
    void addImage(const DRW_Image *data) override {}

	/**
	 * Called for every image definition.
	 */
    void linkImage(const DRW_ImageDef *data) override {}

    /**
     * Called for every comment in the DXF file (code 999).
     */
    void addComment(const char* comment) override {}

    /**
     * Called for PLOTSETTINGS object definition.
     */
    void addPlotSettings(const DRW_PlotSettings *data) override {}

    void writeHeader(DRW_Header& data) override {}
    void writeBlocks() override {}
    void writeBlockRecords() override {}
    void writeEntities() override {}
    void writeLTypes() override {}
    void writeLayers() override {}
    void writeTextstyles() override {}
    void writeVports() override {}
    void writeDimstyles() override {}
    void writeObjects() override {}
    void writeAppId() override {}
private:
    friend class DxfInput;
    DxfInput* _pthis = nullptr;

    dxfRW* _dxfR = nullptr;
    std::vector<common::Point3d> _points;
    std::vector<common::Line3d> _lines;
    std::vector<common::Polygon3d> _polygons;
    std::vector<common::Polyline3d> _polylines;
    std::vector<common::Circled> _circles;
    std::vector<common::Arcd> _arcs;
};
/// DxfInput
DxfInput::DxfInput() {
	if (!_ptr) {
		_ptr = new DxfInputPrivate(this);
	}
}
DxfInput::~DxfInput() {
	if (_ptr) {
		delete _ptr;
		_ptr = nullptr;
	}
}

bool DxfInput::open(const std::string& filepath) {
    return _ptr->open(filepath);
}
bool DxfInput::close() {
	return _ptr->close();
}
const std::vector<common::Point3d>& DxfInput::getPointVec() const {
    return _ptr->_points;
}
const std::vector<common::Line3d>& DxfInput::getLineVec() const {
    return _ptr->_lines;
}
const std::vector<common::Polygon3d>& DxfInput::getPolygonVec() const {
	return _ptr->_polygons;
}
const std::vector<common::Polyline3d>& DxfInput::getPolylineVec() const {
	return _ptr->_polylines;
}
const std::vector<common::Circled>& DxfInput::getCircleVec() const {
    return _ptr->_circles;
}
const std::vector<common::Arcd>& DxfInput::getArcVec() const {
    return _ptr->_arcs;
}

/// DxfOutputPrivate
class DxfOutputPrivate : public DRW_Interface {
public:
    DxfOutputPrivate(DxfOutput* pthis) : _pthis(pthis) {}
    virtual ~DxfOutputPrivate() {}
    bool open(const std::string& filepath) {
        _layers.clear();
        _points.clear();
        _lines.clear();
        _lwpolylines.clear();
        _polylines.clear();
        _texts.clear();
        _images.clear();
        _image_names.clear();
        _dxfW = new dxfRW(filepath.c_str());
        writeLayerId("0", 3); // 默认层
        return true;
    }

    bool close() {
        bool success = _dxfW->write(this, _exportVersion, false);
        delete _dxfW;
        return success;
    }
    void writeLayerIds(const DxfOutput::LayerList& ids, int color) {
        for (const auto& id : ids) {
            this->writeLayerId(id, color);
        }
    }
    void writeLayerId(const std::string& id, int color) {
        DRW_Layer layer;
        layer.name = id;
        layer.color = color; // 3代表绿色
        layer.lWeight = static_cast<DRW_LW_Conv::lineWidth>(1);
        layer.lineType = "CONTINUOUS";
        _layers.push_back(std::move(layer));
    }

    void writePoint3(const common::Point3d& pt, int color, const std::string& layer_id) {
        DRW_Point point;
        setEntityAttributes(&point, color, layer_id);
        point.basePoint.x = pt.x;
        point.basePoint.y = pt.y;
        point.basePoint.z = pt.z;
        _points.push_back(std::move(point));
    }
    void writeLine3(const common::Line3d& line, int color, const std::string& layer_id) {
        DRW_Line l;
        setEntityAttributes(&l, color, layer_id);
        l.basePoint.x = line.begin_point.x;
        l.basePoint.y = line.begin_point.y;
        l.basePoint.z = line.begin_point.z;
        l.secPoint.x = line.end_point.x;
        l.secPoint.y = line.end_point.y;
        l.secPoint.z = line.end_point.z;
        _lines.push_back(std::move(l));
    }
    void writePolyline2(const std::vector<common::Point2d>& points,
            double default_z, bool closed, 
            int color, const std::string& layer_id) {
        DRW_LWPolyline lwpolyline;
        setEntityAttributes(&lwpolyline, color, layer_id);
        lwpolyline.elevation = default_z;
        auto count = points.size();
        for (int i = 0; i < count; ++i) {
            lwpolyline.addVertex(DRW_Vertex2D(points[i].x, points[i].y, 0));
        }
        lwpolyline.flags = (closed ? 1 : 0);
        _lwpolylines.push_back(std::move(lwpolyline));
    }
    void writePolyline3(const std::vector<common::Point3d>& points, bool closed, 
            int color, const std::string& layer_id) {
        DRW_Polyline polyline;
        setEntityAttributes(&polyline, color, layer_id);
        auto count = points.size();
        for (int i = 0; i < count; ++i) {
            polyline.addVertex(DRW_Vertex(points[i].x, points[i].y, points[i].z, 0));
        }
        // 8表示多短线；1表示闭合；8 | 1
        polyline.flags = (closed ? 9 : 8);
        _polylines.push_back(std::move(polyline));
    }

    void writeLayerData(const common::Layerd& layer, int color, const std::string& layer_id) {
        for (auto& point : layer.points) {
            this->writePoint3(point, color, layer_id);
        }
        for (auto& polyline : layer.polylines) {
            this->writePolyline3(polyline.points, false, color, layer_id);
        }
        for (auto& polygon : layer.polygons) {
            this->writePolyline3(polygon.points, true, color, layer_id);
        }
    }
    void writeText(const std::string& str, const common::Line2d& line,
                   double angle, double height, int color, const std::string& layer_id) {
        DRW_Text text;
        setEntityAttributes(&text, color, layer_id);
        text.basePoint.x = line.begin_point.x;
        text.basePoint.y = line.begin_point.y;
        text.basePoint.z = 0.0;
        text.secPoint.x = line.end_point.x;
        text.secPoint.y = line.end_point.y;
        text.secPoint.z = 0.0;
        text.text = str;
        text.height = height;
        text.angle = angle;
        _texts.push_back(std::move(text));
    }
    void writeImage(
            const std::string& filename,
            int width, int height,
            const common::Point3d& insert_point,
            const common::Point3d& uvec,
            const common::Point3d& vvec,
            const std::string& layer_id) {
        DRW_Image image;
        setEntityAttributes(&image, 256, layer_id);
        image.basePoint.x = insert_point.x;
        image.basePoint.y = insert_point.y;
        image.basePoint.z = insert_point.z;
        image.secPoint.x = uvec.x;
        image.secPoint.y = uvec.y;
        image.secPoint.z = uvec.z;
        image.vVector.x = vvec.x;
        image.vVector.y = vvec.y;
        image.vVector.z = vvec.z;
        image.sizeu = width;
        image.sizev = height;
        image.layer = layer_id;
        image.brightness = 50;
        image.contrast = 50;
        image.fade = 0;
        _images.push_back(std::move(image));
        _image_names.push_back(filename);
    }
protected:
    /** Called when header is parsed.  */
    void addHeader(const DRW_Header* data) override {}

    /** Called for every line Type.  */
    void addLType(const DRW_LType& data) override {}
    /** Called for every layer. */
    void addLayer(const DRW_Layer& data) override {}
    /** Called for every dim style. */
    void addDimStyle(const DRW_Dimstyle& data) override {}
    /** Called for every VPORT table. */
    void addVport(const DRW_Vport& data) override {}
    /** Called for every text style. */
    void addTextStyle(const DRW_Textstyle& data) override {}
    /** Called for every AppId entry. */
    void addAppId(const DRW_AppId& data) override {}

    /**
     * Called for every block. Note: all entities added after this
     * command go into this block until endBlock() is called.
     *
     * @see endBlock()
     */
    void addBlock(const DRW_Block& data) override {}

    /**
     * In DWG called when the following entities corresponding to a
     * block different from the current. Note: all entities added after this
     * command go into this block until setBlock() is called already.
     *
     * int handle are the value of DRW_Block::handleBlock added with addBlock()
     */
    void setBlock(const int handle) override {}

    /** Called to end the current block */
    void endBlock() override {}

    /** Called for every point */
    void addPoint(const DRW_Point& data) override {}

    /** Called for every line */
    void addLine(const DRW_Line& data) override {}

    /** Called for every ray */
    void addRay(const DRW_Ray& data) override {}

    /** Called for every xline */
    void addXline(const DRW_Xline& data) override {}

    /** Called for every arc */
    void addArc(const DRW_Arc& data) override {}

    /** Called for every circle */
    void addCircle(const DRW_Circle& data) override {}

    /** Called for every ellipse */
    void addEllipse(const DRW_Ellipse& data) override {}

    /** Called for every lwpolyline */
    void addLWPolyline(const DRW_LWPolyline& data) override {}

    /** Called for every polyline start */
    void addPolyline(const DRW_Polyline& data) override {}

    /** Called for every spline */
    void addSpline(const DRW_Spline* data) override {}
	
	/** Called for every spline knot value */
    void addKnot(const DRW_Entity& data) override {}

    /** Called for every insert. */
    void addInsert(const DRW_Insert& data) override {}
    
    /** Called for every trace start */
    void addTrace(const DRW_Trace& data) override {}
    
    /** Called for every 3dface start */
    void add3dFace(const DRW_3Dface& data) override {}

    /** Called for every solid start */
    void addSolid(const DRW_Solid& data) override {}


    /** Called for every Multi Text entity. */
    void addMText(const DRW_MText& data) override {}

    /** Called for every Text entity. */
    void addText(const DRW_Text& data) override {
        //TODO: 增加文本
    }

    /**
     * Called for every aligned dimension entity. 
     */
    void addDimAlign(const DRW_DimAligned *data) override {}
    /**
     * Called for every linear or rotated dimension entity. 
     */
    void addDimLinear(const DRW_DimLinear *data) override {}

	/**
     * Called for every radial dimension entity. 
     */
    void addDimRadial(const DRW_DimRadial *data) override {}

	/**
     * Called for every diametric dimension entity. 
     */
    void addDimDiametric(const DRW_DimDiametric *data) override {}

	/**
     * Called for every angular dimension (2 lines version) entity. 
     */
    void addDimAngular(const DRW_DimAngular *data) override {}

	/**
     * Called for every angular dimension (3 points version) entity. 
     */
    void addDimAngular3P(const DRW_DimAngular3p *data) override {}
	
    /**
     * Called for every ordinate dimension entity. 
     */
    void addDimOrdinate(const DRW_DimOrdinate *data) override {}
    
    /** 
	 * Called for every leader start. 
	 */
    void addLeader(const DRW_Leader *data) override {}
	
	/** 
	 * Called for every hatch entity. 
	 */
    void addHatch(const DRW_Hatch *data) override {}
	
    /**
     * Called for every viewport entity.
     */
    void addViewport(const DRW_Viewport& data) override {}

    /**
	 * Called for every image entity. 
	 */
    void addImage(const DRW_Image *data) override {}

	/**
	 * Called for every image definition.
	 */
    void linkImage(const DRW_ImageDef *data) override {}

    /**
     * Called for every comment in the DXF file (code 999).
     */
    void addComment(const char* comment) override {}

    /**
     * Called for PLOTSETTINGS object definition.
     */
    void addPlotSettings(const DRW_PlotSettings *data) override {}

    void writeHeader(DRW_Header& data) override {}
    void writeBlocks() override {}
    void writeBlockRecords() override {}
    void writeEntities() override {
        for (auto& point : _points) {
            _dxfW->writePoint(&point);
        }
        for (auto& line : _lines) {
            _dxfW->writeLine(&line);
        }
        for (auto& lwpolyline : _lwpolylines) {
            _dxfW->writeLWPolyline(&lwpolyline);
        }
        for (auto& polyline : _polylines) {
            _dxfW->writePolyline(&polyline);
        }
        for (auto& text : _texts) {
            _dxfW->writeText(&text);
        }
        int image_count = _images.size();
        for (int i = 0; i < image_count; ++i) {
            DRW_ImageDef *imgDef = _dxfW->writeImage(&(_images[i]), _image_names[i]);
            if (imgDef) { // 写出def
                imgDef->loaded = 1;
                imgDef->u = _images[i].sizeu;
                imgDef->v = _images[i].sizev;
                imgDef->up = 1.0;
                imgDef->vp = 1.0;
                imgDef->resolution = 0;
            }
        }
    }
    void writeLTypes() override {
        DRW_LType ltype;
        // Standard linetypes for LibreCAD / AutoCAD
        ltype.name = "CONTINUOUS";
        ltype.desc = "Solid line";
        _dxfW->writeLineType(&ltype);
        ltype.name = "ByLayer";
        _dxfW->writeLineType(&ltype);
        ltype.name = "ByBlock";
        _dxfW->writeLineType(&ltype);

        ltype.name = "DOT";
        ltype.desc = "Dot . . . . . . . . . . . . . . . . . . . . . .";
        ltype.size = 2;
        ltype.length = 6.35;
        ltype.path.push_back(0.0);
        ltype.path.push_back(-6.35);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DOTTINY";
        ltype.desc = "Dot (.15x) .....................................";
        ltype.size = 2;
        ltype.length = 0.9525;
        ltype.path.push_back(0.0);
        ltype.path.push_back(-0.9525);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DOT2";
        ltype.desc = "Dot (.5x) .....................................";
        ltype.size = 2;
        ltype.length = 3.175;
        ltype.path.push_back(0.0);
        ltype.path.push_back(-3.175);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DOTX2";
        ltype.desc = "Dot (2x) .  .  .  .  .  .  .  .  .  .  .  .  .";
        ltype.size = 2;
        ltype.length = 12.7;
        ltype.path.push_back(0.0);
        ltype.path.push_back(-12.7);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHED";
        ltype.desc = "Dashed _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _";
        ltype.size = 2;
        ltype.length = 19.05;
        ltype.path.push_back(12.7);
        ltype.path.push_back(-6.35);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHEDTINY";
        ltype.desc = "Dashed (.15x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _";
        ltype.size = 2;
        ltype.length = 2.8575;
        ltype.path.push_back(1.905);
        ltype.path.push_back(-0.9525);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHED2";
        ltype.desc = "Dashed (.5x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _";
        ltype.size = 2;
        ltype.length = 9.525;
        ltype.path.push_back(6.35);
        ltype.path.push_back(-3.175);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHEDX2";
        ltype.desc = "Dashed (2x) ____  ____  ____  ____  ____  ___";
        ltype.size = 2;
        ltype.length = 38.1;
        ltype.path.push_back(25.4);
        ltype.path.push_back(-12.7);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHDOT";
        ltype.desc = "Dash dot __ . __ . __ . __ . __ . __ . __ . __";
        ltype.size = 4;
        ltype.length = 25.4;
        ltype.path.push_back(12.7);
        ltype.path.push_back(-6.35);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-6.35);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHDOTTINY";
        ltype.desc = "Dash dot (.15x) _._._._._._._._._._._._._._._.";
        ltype.size = 4;
        ltype.length = 3.81;
        ltype.path.push_back(1.905);
        ltype.path.push_back(-0.9525);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-0.9525);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHDOT2";
        ltype.desc = "Dash dot (.5x) _._._._._._._._._._._._._._._.";
        ltype.size = 4;
        ltype.length = 12.7;
        ltype.path.push_back(6.35);
        ltype.path.push_back(-3.175);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-3.175);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DASHDOTX2";
        ltype.desc = "Dash dot (2x) ____  .  ____  .  ____  .  ___";
        ltype.size = 4;
        ltype.length = 50.8;
        ltype.path.push_back(25.4);
        ltype.path.push_back(-12.7);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-12.7);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DIVIDE";
        ltype.desc = "Divide ____ . . ____ . . ____ . . ____ . . ____";
        ltype.size = 6;
        ltype.length = 31.75;
        ltype.path.push_back(12.7);
        ltype.path.push_back(-6.35);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-6.35);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-6.35);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DIVIDETINY";
        ltype.desc = "Divide (.15x) __..__..__..__..__..__..__..__.._";
        ltype.size = 6;
        ltype.length = 4.7625;
        ltype.path.push_back(1.905);
        ltype.path.push_back(-0.9525);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-0.9525);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-0.9525);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DIVIDE2";
        ltype.desc = "Divide (.5x) __..__..__..__..__..__..__..__.._";
        ltype.size = 6;
        ltype.length = 15.875;
        ltype.path.push_back(6.35);
        ltype.path.push_back(-3.175);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-3.175);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-3.175);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "DIVIDEX2";
        ltype.desc = "Divide (2x) ________  .  .  ________  .  .  _";
        ltype.size = 6;
        ltype.length = 63.5;
        ltype.path.push_back(25.4);
        ltype.path.push_back(-12.7);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-12.7);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-12.7);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "BORDER";
        ltype.desc = "Border __ __ . __ __ . __ __ . __ __ . __ __ .";
        ltype.size = 6;
        ltype.length = 44.45;
        ltype.path.push_back(12.7);
        ltype.path.push_back(-6.35);
        ltype.path.push_back(12.7);
        ltype.path.push_back(-6.35);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-6.35);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "BORDERTINY";
        ltype.desc = "Border (.15x) __.__.__.__.__.__.__.__.__.__.__.";
        ltype.size = 6;
        ltype.length = 6.6675;
        ltype.path.push_back(1.905);
        ltype.path.push_back(-0.9525);
        ltype.path.push_back(1.905);
        ltype.path.push_back(-0.9525);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-0.9525);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "BORDER2";
        ltype.desc = "Border (.5x) __.__.__.__.__.__.__.__.__.__.__.";
        ltype.size = 6;
        ltype.length = 22.225;
        ltype.path.push_back(6.35);
        ltype.path.push_back(-3.175);
        ltype.path.push_back(6.35);
        ltype.path.push_back(-3.175);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-3.175);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "BORDERX2";
        ltype.desc = "Border (2x) ____  ____  .  ____  ____  .  ___";
        ltype.size = 6;
        ltype.length = 88.9;
        ltype.path.push_back(25.4);
        ltype.path.push_back(-12.7);
        ltype.path.push_back(25.4);
        ltype.path.push_back(-12.7);
        ltype.path.push_back(0.0);
        ltype.path.push_back(-12.7);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "CENTER";
        ltype.desc = "Center ____ _ ____ _ ____ _ ____ _ ____ _ ____";
        ltype.size = 4;
        ltype.length = 50.8;
        ltype.path.push_back(31.75);
        ltype.path.push_back(-6.35);
        ltype.path.push_back(6.35);
        ltype.path.push_back(-6.35);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "CENTERTINY";
        ltype.desc = "Center (.15x) ___ _ ___ _ ___ _ ___ _ ___ _ ___";
        ltype.size = 4;
        ltype.length = 7.62;
        ltype.path.push_back(4.7625);
        ltype.path.push_back(-0.9525);
        ltype.path.push_back(0.9525);
        ltype.path.push_back(-0.9525);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "CENTER2";
        ltype.desc = "Center (.5x) ___ _ ___ _ ___ _ ___ _ ___ _ ___";
        ltype.size = 4;
        ltype.length = 28.575;
        ltype.path.push_back(19.05);
        ltype.path.push_back(-3.175);
        ltype.path.push_back(3.175);
        ltype.path.push_back(-3.175);
        _dxfW->writeLineType(&ltype);

        ltype.path.clear();
        ltype.name = "CENTERX2";
        ltype.desc = "Center (2x) ________  __  ________  __  _____";
        ltype.size = 4;
        ltype.length = 101.6;
        ltype.path.push_back(63.5);
        ltype.path.push_back(-12.7);
        ltype.path.push_back(12.7);
        ltype.path.push_back(-12.7);
        _dxfW->writeLineType(&ltype);
    }
    void writeLayers() override {
        for (auto& layer : _layers) {
            _dxfW->writeLayer(&layer);
        }
    }
    void writeTextstyles() override {}
    void writeVports() override {}
    void writeDimstyles() override {}
    void writeObjects() override {}
    void writeAppId() override {}

private:
    void setEntityAttributes(DRW_Entity* ent, int color, const std::string& layer_id) {
        ent->layer = layer_id;
        ent->color = color; // 256代表随图层
        ent->lWeight = static_cast<DRW_LW_Conv::lineWidth>(-1);
        ent->lineType = "BYLAYER";
        ent->ltypeScale = 1.0;
    }

private:
    friend class DxfOutput;
    DxfOutput* _pthis = nullptr;

    dxfRW* _dxfW = nullptr;
    DRW::Version _exportVersion = DRW::AC1027;

    std::list<DRW_Layer> _layers;
    std::list<DRW_Point> _points;
    std::list<DRW_Line> _lines;
    std::list<DRW_LWPolyline> _lwpolylines;
    std::list<DRW_Polyline> _polylines;
    std::vector<DRW_Image> _images;
    std::vector<DRW_Text> _texts;
    std::vector<std::string> _image_names;
};

/// DxfOutput
DxfOutput::DxfOutput() {
    if (!_ptr) {
        _ptr = new DxfOutputPrivate(this);
    }
}
DxfOutput::~DxfOutput() {
    if (_ptr) {
        delete _ptr;
        _ptr = nullptr;
    }
}

bool DxfOutput::open(const std::string& filename) {
    return _ptr->open(filename);
}
bool DxfOutput::close() {
    return _ptr->close();
}

void DxfOutput::writeLayerIds(const LayerList& layers, int color) {
    return _ptr->writeLayerIds(layers, color);
}

void DxfOutput::writePoint3(const common::Point3d& pt, int color, 
        const std::string& layer_id) {
    return _ptr->writePoint3(pt, color, layer_id);
}

void DxfOutput::writeLine3(const common::Line3d &line, int color, const std::string &layer_id) {
    return _ptr->writeLine3(line, color, layer_id);
}
void DxfOutput::writePolyline2(
        const std::vector<common::Point2d>& points, double default_z, bool closed, 
        int color, const std::string& layer_id) {
    return _ptr->writePolyline2(points, default_z, closed, color, layer_id);
}
void DxfOutput::writePolyline3(
        const std::vector<common::Point3d>& points, bool closed, 
        int color, const std::string& layer_id) {
    return _ptr->writePolyline3(points, closed, color, layer_id);
}

void DxfOutput::writeLayer(const common::Layerd& layer, int color, const std::string& layer_id) {
    return _ptr->writeLayerData(layer, color, layer_id);
}
void DxfOutput::writeText(const std::string& str, const common::Line2d& line,
        double angle, double height, int color, const std::string& layer_id) {
    return _ptr->writeText(str, line, angle, height, color, layer_id);
}
void DxfOutput::writeImage(
        const std::string& filename,
        int width, int height,
        const common::Point3d& insert_point,
        const common::Point3d& uvec,
        const common::Point3d& vvec,
        const std::string& layer_id) {
    return _ptr->writeImage(filename, width, height, insert_point, uvec, vvec, layer_id);
}
}
