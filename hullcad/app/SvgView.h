#pragma once
// Zoomable SVG viewer: mouse wheel zooms, drag pans, double-click fits the drawing.
#include <QByteArray>
#include <QGraphicsScene>
#include <QGraphicsView>

class QGraphicsSvgItem;
class QSvgRenderer;

class SvgView : public QGraphicsView {
    Q_OBJECT
public:
    explicit SvgView(QWidget* parent = nullptr);
    void setSvg(const QByteArray& svg);
    void clear();
    QByteArray svg() const { return data_; }

protected:
    void wheelEvent(QWheelEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;

private:
    void fit();

    QGraphicsScene scene_;
    QSvgRenderer* renderer_ = nullptr;
    QGraphicsSvgItem* item_ = nullptr;
    QByteArray data_;
    bool fitted_ = true;
};
