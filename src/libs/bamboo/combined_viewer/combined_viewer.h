#pragma once
#include <QSplitter>
#include <QUndoStack>
#include <uface/canvas2d/view.hpp>
#include "bamboo/osg_viewer/osg_viewer.h"
#include "bamboo/combined_viewer/combined_shape_layer.h"

namespace welkin::bamboo {
using Viewer2d = uface::canvas2d::Viewer;
using Viewer3d = OSGViewer;
using Layer2d = uface::canvas2d::Layer;
using Layer3d = OSGLayer;
// 二维三维联合视图
class BAMBOO_EXPORT CombinedViewer : public QSplitter {
    Q_OBJECT
public:
    CombinedViewer(QWidget* parent = nullptr);
    virtual ~CombinedViewer();

    Viewer2d* viewer2d() const;
    Viewer3d* viewer3d() const;

    // 设置场景z值
    void setSceneZValue(double scene_z);
    // 设置投影坐标
    void setProjectPose(const common::Pose3d& pose);

    // 是否联动
    bool isInteraction() const;
    // 获取联动动作
    QAction* getInteractionAction() const;
    // 是否显示所有
    bool isShowAll() const;
    QAction* getShowAllAction() const;

    // 图层CRUD
    bool hasLayer2d(const QString& name) const;
    Layer2d* getLayer2d(const QString& name) const;
    QMap<QString, Layer2d*> getLayer2dMap() const;
    bool hasLayer3d(const QString& name) const;
    Layer3d* getLayer3d(const QString& name) const;
    QMap<QString, Layer3d*> getLayer3dMap() const;
    bool hasCombinedShapeLayer(const QString& name) const;
    CombinedShapeLayer* getCombinedShapeLayer(const QString& name) const;
    QMap<QString, CombinedShapeLayer*> getCombinedShapeLayerMap() const;
    void getLayers(QList<Layer2d*>& layers) const;
    void getLayers(QList<Layer3d*>& layers) const;
    void getLayers(QList<CombinedShapeLayer*>& layers) const;

    void addLayer(Layer2d* layer);
    void addLayers(const QList<Layer2d*>& layers);
    void addLayer(Layer3d* layer);
    void addLayers(const QList<Layer3d*>& layers);
    void addLayer(CombinedShapeLayer* layer);
    void addLayers(const QList<CombinedShapeLayer*>& layers);
    void removeLayer(const QString& name);
    void removeLayers(const QList<QString>& names);
    void clearLayers(int type);

Q_SIGNALS:
    // 0 - layer2d; 1 - layer3d; 2 - combinedlayer
    void layerAdded(int type, const QString& name);
    void layersAdded(int type, const QList<QString>& names);
    void layerRemoved(int type, const QString& name);
    void layersRemoved(int type, const QList<QString>& name);
    void layersCleared(int type);
    void projectPoseChanged();
public Q_SLOTS:
    void onFitInView();
    void onFitInView(const QRectF& brect);

protected Q_SLOTS:
    void onSceneScaledSlot(double scale);
    void onSceneMovedSlot(const QPointF& cnt_pt, double scale);
    void onShowAllSlot();
    void onChangeProjectPoseSlot();
protected:
    void addLayerHelper(Layer2d* layer);
    void addLayerHelper(Layer3d* layer);
    void addLayerHelper(CombinedShapeLayer* layer);
    void removeLayer2dHelper(const QString& name);
    void removeLayer3dHelper(const QString& name);
    void removeCombinedShapeLayerHelper(const QString& name);
    void createActions();
    
protected:
    Viewer2d* _viewer2d = nullptr;
    Viewer3d* _viewer3d = nullptr;

    double _scene_z = 0.0;
    common::Pose3d _proj_pose;
    QAction* _interaction_action = nullptr;
    bool _show_all = true; // 显示所有
    QAction* _show_all_action = nullptr;

    // 图层信息
    QMap<QString, Layer2d*> _layer2d_map;
    QMap<QString, Layer3d*> _layer3d_map;
    QMap<QString, CombinedShapeLayer*> _combined_layer_map;
};
}
