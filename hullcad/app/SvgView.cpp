#include "SvgView.h"

#include <QGraphicsSvgItem>
#include <QResizeEvent>
#include <QSvgRenderer>
#include <QWheelEvent>

SvgView::SvgView(QWidget* parent) : QGraphicsView(parent), renderer_(new QSvgRenderer(this)) {
    setScene(&scene_);
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform | QPainter::TextAntialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setBackgroundBrush(QColor(0xfb, 0xfa, 0xf7));
    setFrameShape(QFrame::NoFrame);
}

void SvgView::clear() {
    scene_.clear();
    item_ = nullptr;
    data_.clear();
}

void SvgView::setSvg(const QByteArray& svg) {
    clear();
    data_ = svg;
    if (!renderer_->load(svg)) return;
    item_ = new QGraphicsSvgItem();
    item_->setSharedRenderer(renderer_);
    scene_.addItem(item_);
    scene_.setSceneRect(item_->boundingRect());
    fitted_ = true;
    fit();
}

void SvgView::fit() {
    if (item_) fitInView(item_, Qt::KeepAspectRatio);
}

void SvgView::wheelEvent(QWheelEvent* e) {
    const double f = e->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2;
    scale(f, f);
    fitted_ = false;
}

void SvgView::resizeEvent(QResizeEvent* e) {
    QGraphicsView::resizeEvent(e);
    if (fitted_) fit();
}

void SvgView::mouseDoubleClickEvent(QMouseEvent*) {
    fitted_ = true;
    fit();
}
