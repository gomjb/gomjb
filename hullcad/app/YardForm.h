#pragma once
// Yard profile editor generated from the option registry (hullcad::yard_options): one group per
// INI section, one widget per option. Adding an option to the registry adds it to this form.
#include <QMap>
#include <QWidget>

#include "hullcad/yard.hpp"

class QGroupBox;

class YardForm : public QWidget {
    Q_OBJECT
public:
    explicit YardForm(QWidget* parent = nullptr);

    void setProfile(const hullcad::YardProfile& y);
    // Builds a profile from the widgets; throws std::invalid_argument on malformed list values.
    hullcad::YardProfile profile() const;

signals:
    void changed();

private:
    QMap<QString, QWidget*> widgets_;              // option key -> editor
    QMap<QString, QGroupBox*> groups_;             // section -> group
    void updateEnabledState(const QString& section);
};
