#include "YardForm.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>

using hullcad::YardOption;
using hullcad::YardProfile;

YardForm::YardForm(QWidget* parent) : QWidget(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);
    YardProfile defaults;
    auto opts = hullcad::yard_options(defaults);
    for (auto& [sec, desc] : hullcad::yard_sections()) {
        const QString section = QString::fromStdString(sec);
        auto* box = new QGroupBox(QStringLiteral("[%1]").arg(section), this);
        auto* form = new QFormLayout(box);
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setLabelAlignment(Qt::AlignLeft);
        auto* about = new QLabel(QString::fromUtf8(desc.c_str()), box);
        about->setWordWrap(true);
        about->setStyleSheet(QStringLiteral("color: #1d3557; font-weight: 600;"));
        form->addRow(about);
        for (auto& o : opts) {
            if (o.key.compare(0, sec.size() + 1, sec + ".") != 0) continue;
            const QString key = QString::fromStdString(o.key);
            const QString name = QString::fromStdString(o.key.substr(sec.size() + 1));
            const QString unit = (o.unit.empty() || o.unit == "-") ? QString() : QString::fromStdString(o.unit);
            const QString help = QString::fromUtf8(o.help.c_str());
            QWidget* w = nullptr;
            switch (o.type) {
                case YardOption::Type::Bool: {
                    auto* c = new QCheckBox(tr("사용"), box);
                    connect(c, &QCheckBox::toggled, this, [this, section] {
                        updateEnabledState(section);
                        emit changed();
                    });
                    w = c;
                    break;
                }
                case YardOption::Type::Number: {
                    auto* s = new QDoubleSpinBox(box);
                    s->setRange(0.0, 1.0e6);
                    s->setDecimals(3);
                    s->setSingleStep(unit == "t" ? 10.0 : (unit == "m" ? 0.5 : 0.01));
                    if (!unit.isEmpty()) s->setSuffix(QStringLiteral(" ") + unit);
                    s->setAccelerated(true);
                    connect(s, &QDoubleSpinBox::valueChanged, this, &YardForm::changed);
                    w = s;
                    break;
                }
                case YardOption::Type::Integer: {
                    auto* s = new QSpinBox(box);
                    s->setRange(0, 100000);
                    if (!unit.isEmpty()) s->setSuffix(QStringLiteral(" ") + unit);
                    connect(s, &QSpinBox::valueChanged, this, &YardForm::changed);
                    w = s;
                    break;
                }
                case YardOption::Type::Text:
                case YardOption::Type::List:
                case YardOption::Type::NumberList: {
                    auto* e = new QLineEdit(box);
                    if (o.type != YardOption::Type::Text) e->setPlaceholderText(tr("쉼표로 구분"));
                    connect(e, &QLineEdit::textEdited, this, &YardForm::changed);
                    w = e;
                    break;
                }
            }
            w->setToolTip(QStringLiteral("%1\n%2%3").arg(key, help, unit.isEmpty() ? QString() : QStringLiteral(" [%1]").arg(unit)));
            {
                auto* label = new QLabel(name, box);
                label->setToolTip(w->toolTip());
                auto* row = new QWidget(box);
                auto* rl = new QVBoxLayout(row);
                rl->setContentsMargins(0, 0, 0, 0);
                rl->setSpacing(0);
                rl->addWidget(w);
                auto* hint = new QLabel(help, row);
                hint->setWordWrap(true);
                hint->setStyleSheet(QStringLiteral("color: #777; font-size: 11px;"));
                rl->addWidget(hint);
                form->addRow(label, row);
            }
            widgets_.insert(key, w);
        }
        groups_.insert(section, box);
        outer->addWidget(box);
    }
    outer->addStretch(1);
    setProfile(defaults);
}

void YardForm::setProfile(const YardProfile& yin) {
    YardProfile y = yin;
    for (auto& o : hullcad::yard_options(y)) {
        QWidget* w = widgets_.value(QString::fromStdString(o.key));
        if (!w) continue;
        const QString v = QString::fromStdString(hullcad::get_yard_option(y, o.key));
        const QSignalBlocker block(w);
        if (auto* c = qobject_cast<QCheckBox*>(w)) c->setChecked(v == QStringLiteral("true"));
        else if (auto* d = qobject_cast<QDoubleSpinBox*>(w)) d->setValue(v.toDouble());
        else if (auto* i = qobject_cast<QSpinBox*>(w)) i->setValue(v.toInt());
        else if (auto* e = qobject_cast<QLineEdit*>(w)) e->setText(v);
    }
    for (auto it = groups_.begin(); it != groups_.end(); ++it) updateEnabledState(it.key());
}

YardProfile YardForm::profile() const {
    YardProfile y;
    for (auto it = widgets_.begin(); it != widgets_.end(); ++it) {
        QString v;
        QWidget* w = it.value();
        if (auto* c = qobject_cast<QCheckBox*>(w)) v = c->isChecked() ? QStringLiteral("true") : QStringLiteral("false");
        else if (auto* d = qobject_cast<QDoubleSpinBox*>(w)) v = QString::number(d->value(), 'g', 12);
        else if (auto* i = qobject_cast<QSpinBox*>(w)) v = QString::number(i->value());
        else if (auto* e = qobject_cast<QLineEdit*>(w)) v = e->text();
        hullcad::set_yard_option(y, it.key().toStdString(), v.toStdString());
    }
    return y;
}

void YardForm::updateEnabledState(const QString& section) {
    // A stage switched off greys out its limits.
    auto* en = qobject_cast<QCheckBox*>(widgets_.value(section + QStringLiteral(".enabled")));
    if (!en) return;
    for (auto it = widgets_.begin(); it != widgets_.end(); ++it)
        if (it.key().startsWith(section + QStringLiteral(".")) && it.value() != en) it.value()->setEnabled(en->isChecked());
}
