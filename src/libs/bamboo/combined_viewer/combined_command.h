#pragma once
#include <QUndoCommand>
#include "bamboo/combined_viewer/combined_viewer.h"

namespace welkin::bamboo {
/// Add layer
template <class LayerT>
class BAMBOO_EXPORT TypeLayerAddCommand : public QUndoCommand {
public:
    TypeLayerAddCommand(CombinedViewer* viewer,
        LayerT* layer, QUndoCommand *parent = 0)
        : QUndoCommand(parent), _viewer(viewer), _layer(layer) {}
    virtual ~TypeLayerAddCommand() {}

    void undo() override {
        if (_viewer && _layer) {
            auto name = _layer->name();
            _viewer->removeLayer(name);
            setText(QObject::tr("Undo add layer"));
        }
    }
    void redo() override {
        if (_viewer && _layer) {
            _viewer->addLayer(_layer);
            setText(QObject::tr("Redo add layer"));
        }
    }

private:
    CombinedViewer* _viewer = nullptr;
    LayerT* _layer = nullptr;
};
using Layer2dAddCommand = TypeLayerAddCommand<Layer2d>;
using Layer3dAddCommand = TypeLayerAddCommand<Layer3d>;
using CombinedShapeLayerAddCommand = TypeLayerAddCommand<CombinedShapeLayer>;

/// Add layers
template <class LayerT>
class TypeLayersAddCommand : public QUndoCommand {
public:
    TypeLayersAddCommand(CombinedViewer* viewer,
        const QList<LayerT*>& layers, QUndoCommand *parent = 0)
        : QUndoCommand(parent), _viewer(viewer), _layers(layers) {}
    virtual ~TypeLayersAddCommand() {}

    void undo() override {
        if (_viewer) {
            QList<QString> names;
            for (auto& layer : _layers) {
                names.append(layer->name());
            }
            _viewer->removeLayers(names);
            setText(QObject::tr("Undo add layers"));
        }
    }
    void redo() override {
        if (_viewer) {
            _viewer->addLayers(_layers);
            setText(QObject::tr("Redo add layers"));
        }
    }

private:
    CombinedViewer* _viewer = nullptr;
    QList<LayerT*> _layers;
};
using Layers2dAddCommand = TypeLayersAddCommand<Layer2d>;
using Layers3dAddCommand = TypeLayersAddCommand<Layer3d>;
using CombinedShapeLayersAddCommand = TypeLayersAddCommand<CombinedShapeLayer>;

/// Remove layer
template <class LayerT>
class TypeLayerRemoveCommand : public QUndoCommand {
public:
    TypeLayerRemoveCommand(CombinedViewer* viewer,
        LayerT* layer, QUndoCommand *parent = 0)
        : QUndoCommand(parent), _viewer(viewer), _layer(layer) {}
    virtual ~TypeLayerRemoveCommand() {}

    void undo() override {
        if (_viewer && _layer) {
            _viewer->addLayer(_layer);
            setText(QObject::tr("Undo remove layer"));
        }
    }
    void redo() override {
        if (_viewer && _layer) {
            auto name = _layer->name();
            _viewer->removeLayer(name);
            setText(QObject::tr("Redo remove layer"));
        }
    }

private:
    CombinedViewer* _viewer = nullptr;
    LayerT* _layer = nullptr;
};
using Layer2dRemoveCommand = TypeLayerRemoveCommand<Layer2d>;
using Layer3dRemoveCommand = TypeLayerRemoveCommand<Layer3d>;
using CombinedShapeLayerRemoveCommand = TypeLayerRemoveCommand<CombinedShapeLayer>;

/// Remove layers
template <class LayerT>
class TypeLayersRemoveCommand : public QUndoCommand {
public:
    TypeLayersRemoveCommand(CombinedViewer* viewer,
        const QList<LayerT*>& layers, QUndoCommand *parent = 0)
        : QUndoCommand(parent), _viewer(viewer), _layers(layers) {}
    virtual ~TypeLayersRemoveCommand() {}

    void undo() override {
        if (_viewer) {
            QList<QString> names;
            for (auto& layer : _layers) {
                names.append(layer->name());
            }
            _viewer->removeLayers(names);
            setText(QObject::tr("Undo add layers"));
        }
    }
    void redo() override {
        if (_viewer) {
            _viewer->addLayers(_layers);
            setText(QObject::tr("Redo add layers"));
        }
    }

private:
    CombinedViewer* _viewer = nullptr;
    QList<LayerT*> _layers;
};
using Layers2dRemoveCommand = TypeLayersRemoveCommand<Layer2d>;
using Layers3dRemoveCommand = TypeLayersRemoveCommand<Layer3d>;
using CombinedShapeLayersRemoveCommand = TypeLayersRemoveCommand<CombinedShapeLayer>;

/// Clear layers
template <class LayerT, int Key>
class TypeLayersClearCommand : public QUndoCommand {
public:
    TypeLayersClearCommand(CombinedViewer* viewer, QUndoCommand *parent = 0)
            : QUndoCommand(parent), _viewer(viewer) {
        _viewer->getLayers(_layers);
    }
    virtual ~TypeLayersClearCommand() {}

    void undo() override {
        if (_viewer) {
            _viewer->addLayers(_layers);
            setText(QObject::tr("Undo clear layers"));
        }
    }
    void redo() override {
        if (_viewer) {
            _viewer->clearLayers(Key);
            setText(QObject::tr("Redo clear layers"));
        }
    }

private:
    CombinedViewer* _viewer = nullptr;
    QList<LayerT*> _layers;
};
using Layers2dClearCommand = TypeLayersClearCommand<Layer2d, 0>;
using Layers3dClearCommand = TypeLayersClearCommand<Layer3d, 1>;
using CombinedShapeLayersClearCommand = TypeLayersClearCommand<CombinedShapeLayer, 2>;
}
