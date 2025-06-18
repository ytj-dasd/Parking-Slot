#include "combined_viewer.h"
#include "combined_command.h"
#include <uface/base/uconverter.h>

namespace welkin::bamboo {
const int Layer2dKey = 0;
const int Layer3dKey = 1;
const int CombinedShapeLayerKey = 2;
CombinedViewer::CombinedViewer(QWidget *parent) : QSplitter(parent) {
    _viewer2d = new Viewer2d(this);
    _viewer2d->setYFlipped(true); // 将Y反向
    _viewer3d = new Viewer3d(this);
    this->addWidget(_viewer3d);
    this->addWidget(_viewer2d);
    this->setSizes(QList<int>() << 100 << 100); // 等宽
    this->createActions(); // 创建动作

    connect(_viewer2d, SIGNAL(sceneMoved(const QPointF&, double)),
            this, SLOT(onSceneMovedSlot(const QPointF&, double)));
    connect(_viewer2d, SIGNAL(sceneScaled(double)),
            this, SLOT(onSceneScaledSlot(double)));
    connect(this, SIGNAL(projectPoseChanged()), this, SLOT(onChangeProjectPoseSlot()));
}

CombinedViewer::~CombinedViewer() {}

Viewer2d* CombinedViewer::viewer2d() const {
    return _viewer2d;
}

Viewer3d* CombinedViewer::viewer3d() const {
    return _viewer3d;
}

void CombinedViewer::setSceneZValue(double scene_z) {
    _scene_z = scene_z;
}

void CombinedViewer::setProjectPose(const common::Pose3d &pose) {
    if (_proj_pose.isEqualTo(pose)) {return;}
    _proj_pose = pose;
    emit projectPoseChanged();
}

bool CombinedViewer::isInteraction() const {
    return _interaction_action->isChecked();
}

QAction *CombinedViewer::getInteractionAction() const {
    return _interaction_action;
}

bool CombinedViewer::isShowAll() const {
    return _show_all;
}

QAction *CombinedViewer::getShowAllAction() const {
    return _show_all_action;
}

bool CombinedViewer::hasLayer2d(const QString &name) const {
    return _layer2d_map.contains(name);
}

Layer2d *CombinedViewer::getLayer2d(const QString &name) const {
    return hasLayer2d(name) ? _layer2d_map[name] : nullptr;
}

QMap<QString, Layer2d *> CombinedViewer::getLayer2dMap() const {
    return _layer2d_map;
}

bool CombinedViewer::hasLayer3d(const QString &name) const {
    return _layer3d_map.contains(name);
}

Layer3d *CombinedViewer::getLayer3d(const QString &name) const {
    return hasLayer3d(name) ? _layer3d_map[name] : nullptr;
}

QMap<QString, Layer3d *> CombinedViewer::getLayer3dMap() const {
    return _layer3d_map;
}

bool CombinedViewer::hasCombinedShapeLayer(const QString &name) const {
    return _combined_layer_map.contains(name);
}

CombinedShapeLayer *CombinedViewer::getCombinedShapeLayer(const QString &name) const {
    return hasCombinedShapeLayer(name) ? _combined_layer_map[name] : nullptr;
}

QMap<QString, CombinedShapeLayer *> CombinedViewer::getCombinedShapeLayerMap() const {
    return _combined_layer_map;
}

void CombinedViewer::getLayers(QList<Layer2d *> &layers) const {
    layers = _layer2d_map.values();
}

void CombinedViewer::getLayers(QList<Layer3d *> &layers) const {
    layers = _layer3d_map.values();
}

void CombinedViewer::getLayers(QList<CombinedShapeLayer *> &layers) const {
    layers = _combined_layer_map.values();
}

void CombinedViewer::addLayer(Layer2d *layer) {
    if (!layer || hasLayer2d(layer->name())) {return;}
    this->addLayerHelper(layer);
    emit layerAdded(Layer2dKey, layer->name());
}

void CombinedViewer::addLayers(const QList<Layer2d *> &layers) {
    QList<Layer2d*> valid_layers;
    for (auto& layer : layers) {
        if (!layer || hasLayer2d(layer->name())) {continue;}
        valid_layers.append(layer);
    }
    if (valid_layers.empty()) {return;}
    QList<QString> names;
    for (auto& layer : valid_layers) {
        this->addLayerHelper(layer);
        names.append(layer->name());
    }
    emit layersAdded(Layer2dKey, names);
}

void CombinedViewer::addLayer(Layer3d *layer) {
    if (!layer || hasLayer3d(layer->name())) {return;}
    this->addLayerHelper(layer);
    emit layerAdded(Layer3dKey, layer->name());
}

void CombinedViewer::addLayers(const QList<Layer3d *> &layers) {
    QList<Layer3d*> valid_layers;
    for (auto& layer : layers) {
        if (!layer || hasLayer2d(layer->name())) {continue;}
        valid_layers.append(layer);
    }
    if (valid_layers.empty()) {return;}
    QList<QString> names;
    for (auto& layer : valid_layers) {
        this->addLayerHelper(layer);
        names.append(layer->name());
    }
    emit layersAdded(Layer3dKey, names);
}

void CombinedViewer::addLayer(CombinedShapeLayer *layer) {
    if (!layer || hasCombinedShapeLayer(layer->name())) {return;}
    this->addLayerHelper(layer);
    emit layerAdded(CombinedShapeLayerKey, layer->name());
}

void CombinedViewer::addLayers(const QList<CombinedShapeLayer *> &layers) {
    QList<CombinedShapeLayer*> valid_layers;
    for (auto& layer : layers) {
        if (!layer || hasLayer2d(layer->name())) {continue;}
        valid_layers.append(layer);
    }
    if (valid_layers.empty()) {return;}
    QList<QString> names;
    for (auto& layer : valid_layers) {
        this->addLayerHelper(layer);
        names.append(layer->name());
    }
    emit layersAdded(CombinedShapeLayerKey, names);
}

void CombinedViewer::removeLayer(const QString &name) {
    // 不包含图层
    if (!hasLayer2d(name) && !hasLayer3d(name) && !hasCombinedShapeLayer(name)) {
        return;
    }
    if (hasLayer2d(name)) {
        this->removeLayer2dHelper(name);
        emit layerRemoved(Layer2dKey, name);
    } else if (hasLayer3d(name)) {
        this->removeLayer3dHelper(name);
        emit layerRemoved(Layer3dKey, name);
    } else {
        this->removeCombinedShapeLayerHelper(name);
        emit layerRemoved(CombinedShapeLayerKey, name);
    }
}

void CombinedViewer::removeLayers(const QList<QString> &names) {
    QList<QString> layer2d_names, layer3d_names, combined_layer_names;
    for (auto& name : names) {
        if (hasLayer2d(name)) {
            layer2d_names.append(name);
        } else if (hasLayer3d(name)) {
            layer3d_names.append(name);
        } else if (hasCombinedShapeLayer(name)) {
            combined_layer_names.append(name);
        }
    }
    if(!layer2d_names.empty()) {
        for (auto& name : layer2d_names) {
            this->removeLayer2dHelper(name);
        }
        emit layersRemoved(Layer2dKey, layer2d_names);
    }
    if (!layer3d_names.empty()) {
        for (auto& name : layer3d_names) {
            this->removeLayer3dHelper(name);
        }
        emit layersRemoved(Layer3dKey, layer3d_names);
    }
    if (!combined_layer_names.empty()) {
        for (auto& name : combined_layer_names) {
            this->removeCombinedShapeLayerHelper(name);
        }
        emit layersRemoved(CombinedShapeLayerKey, combined_layer_names);
    }
}

void CombinedViewer::clearLayers(int type) {
    if (type == Layer2dKey) {
        auto names = _layer2d_map.keys();
        for (auto& name : names) {
            this->removeLayer2dHelper(name);
        }
        emit layersCleared(Layer2dKey);
    } else if (type == Layer3dKey) {
        auto names = _layer3d_map.keys();
        for (auto& name : names) {
            this->removeLayer3dHelper(name);
        }
        emit layersCleared(Layer3dKey);
    } else if (type == CombinedShapeLayerKey) {
        auto names = _combined_layer_map.keys();
        for (auto& name : names) {
            this->removeCombinedShapeLayerHelper(name);
        }
        emit layersCleared(CombinedShapeLayerKey);
    }
}

/// public slots
void CombinedViewer::onFitInView() {
    _viewer2d->onFitVisibleItemsInView();
    auto cnt_pt = _viewer2d->getViewCenter();
    double scale = _viewer2d->getViewScale();
    this->onSceneMovedSlot(cnt_pt, scale);
}

void CombinedViewer::onFitInView(const QRectF &brect) {
    _viewer2d->onFitInView(brect);
    auto cnt_pt = _viewer2d->getViewCenter();
    double scale = _viewer2d->getViewScale();
    this->onSceneMovedSlot(cnt_pt, scale);
}

/// protected slots
void CombinedViewer::onSceneScaledSlot(double scale) {
    QPointF cnt_pt = _viewer2d->getViewCenter();
    this->onSceneMovedSlot(cnt_pt, scale);
}

void CombinedViewer::onSceneMovedSlot(const QPointF &cnt_pt, double scale) {
    // 不联动或三维视口不可见，退出
    if (!isInteraction() || !_viewer3d->isVisible()) {return;}

    double view_height = _viewer3d->height();
    double fovy = _viewer3d->getCameraFovY();
    double cam_height = (view_height * scale * 0.5) / std::tan(fovy / 2);

    auto mat = _proj_pose.toMatrix();
    common::Point3d obj_pt(cnt_pt.x(), cnt_pt.y(), _scene_z);
    common::Point3d cam_pt(cnt_pt.x(), cnt_pt.y(), _scene_z + cam_height);
    obj_pt.transform(mat);
    cam_pt.transform(mat);
    common::Point3d updir_pt(mat(0, 1), mat(1, 1), mat(2, 1));
    _viewer3d->setCameraPosition(cam_pt, obj_pt, updir_pt);
}

void CombinedViewer::onShowAllSlot() {
    _show_all = !_show_all; // 取反
    _viewer3d->setVisible(_show_all);
    if (_show_all) {
        _show_all_action->setText(tr("Show Only 2D Viewer"));
        QIcon icon(":/bamboo/images/zoom_in.png");
        _show_all_action->setIcon(icon);
    } else {
        _show_all_action->setText(tr("Show All"));
        QIcon icon(":/bamboo/images/zoom_out.png");
        _show_all_action->setIcon(icon);
    }
}

void CombinedViewer::onChangeProjectPoseSlot() {
    for (auto& layer : _combined_layer_map) {
        layer->setProjectPose(_proj_pose);
    }
}

void CombinedViewer::addLayerHelper(Layer2d *layer) {
    _viewer2d->addLayer(layer);
    _layer2d_map.insert(layer->name(), layer);
}

void CombinedViewer::addLayerHelper(Layer3d *layer) {
    _viewer3d->addLayer(layer);
    _layer3d_map.insert(layer->name(), layer);
}

void CombinedViewer::addLayerHelper(CombinedShapeLayer *layer) {
    _viewer2d->addLayer(layer->layer2d());
    _viewer3d->addLayer(layer->layer3d());
    layer->setProjectPose(_proj_pose);
    _combined_layer_map.insert(layer->name(), layer);
}

void CombinedViewer::removeLayer2dHelper(const QString &name) {
    _viewer2d->removeLayer(name);
    _layer2d_map.remove(name);
}

void CombinedViewer::removeLayer3dHelper(const QString &name) {
    _viewer3d->removeLayer(UFQ(name));
    _layer3d_map.remove(name);
}

void CombinedViewer::removeCombinedShapeLayerHelper(const QString &name) {
    _viewer2d->removeLayer(name);
    _viewer3d->removeLayer(UFQ(name));
    _combined_layer_map.remove(name);
}

/// protected functions
void CombinedViewer::createActions() {
    // 联动动作
    _interaction_action = new QAction(tr("Interaction"), this);
    QIcon interaction_icon(":/bamboo/images/interaction.png");
    _interaction_action->setIcon(interaction_icon);
    _interaction_action->setCheckable(true);
    _interaction_action->setChecked(true);

    // 将二维视口全屏
    _show_all_action = new QAction(tr("Show Only 2D Viewer"), this);
    QIcon show_all_icon(":/bamboo/images/zoom_in.png");
    _show_all_action->setIcon(show_all_icon);
    connect(_show_all_action, SIGNAL(triggered()), this, SLOT(onShowAllSlot()));
}
}
