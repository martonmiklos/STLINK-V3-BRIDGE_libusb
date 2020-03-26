#ifndef GPIOBITSETTINGSWIDGET_H
#define GPIOBITSETTINGSWIDGET_H

#include "bridge.h"

#include <QWidget>

namespace Ui {
class GPIOBitSettingsWidget;
}

class GPIOBitSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit GPIOBitSettingsWidget(QWidget *parent = nullptr);
    ~GPIOBitSettingsWidget();
    void setIndex(quint8 name);
    Brg_GpioModeT mode() const;
    Brg_GpioSpeedT speed() const;
    Brg_GpioPullT pull() const;
    Brg_GpioOutputT outputType() const;

    quint8 index() const;
    void setValue(Brg_GpioValT value);
    Brg_GpioValT value() const;

private slots:
    void on_comboBoxDirection_currentIndexChanged(int index);

private:
    Ui::GPIOBitSettingsWidget *ui;
    quint8 m_index = 0;
};

#endif // GPIOBITSETTINGSWIDGET_H
