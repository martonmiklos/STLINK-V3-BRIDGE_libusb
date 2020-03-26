#include "gpiobitsettingswidget.h"
#include "ui_gpiobitsettingswidget.h"

#include "bridge.h"

GPIOBitSettingsWidget::GPIOBitSettingsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GPIOBitSettingsWidget)
{
    ui->setupUi(this);
    on_comboBoxDirection_currentIndexChanged(ui->comboBoxDirection->currentIndex());
}

GPIOBitSettingsWidget::~GPIOBitSettingsWidget()
{
    delete ui;
}

void GPIOBitSettingsWidget::setIndex(quint8 name)
{
    ui->labelGPIOBitName->setText(QString::number(name));
    m_index = name;
}

Brg_GpioModeT GPIOBitSettingsWidget::mode() const
{
    return static_cast<Brg_GpioModeT>(ui->comboBoxDirection->currentIndex());
}

Brg_GpioSpeedT GPIOBitSettingsWidget::speed() const
{
    return static_cast<Brg_GpioSpeedT>(ui->comboBoxOutputSpeed->currentIndex());
}

Brg_GpioPullT GPIOBitSettingsWidget::pull() const
{
    return static_cast<Brg_GpioPullT>(ui->comboBoxInputPull->currentIndex());
}

Brg_GpioOutputT GPIOBitSettingsWidget::outputType() const
{
    return static_cast<Brg_GpioOutputT>(ui->comboBoxOutputType->currentIndex());
}

void GPIOBitSettingsWidget::on_comboBoxDirection_currentIndexChanged(int index)
{
    ui->groupBoxInputPull->setVisible(index == GPIO_MODE_INPUT);
    ui->groupBoxOutputType->setVisible(index == GPIO_MODE_OUTPUT);
    ui->groupBoxSpeed->setVisible(index == GPIO_MODE_OUTPUT);
    ui->comboBoxValue->setEnabled(index == GPIO_MODE_OUTPUT);
}

quint8 GPIOBitSettingsWidget::index() const
{
    return m_index;
}

void GPIOBitSettingsWidget::setValue(Brg_GpioValT value)
{
    ui->comboBoxValue->setCurrentIndex(value);
}

Brg_GpioValT GPIOBitSettingsWidget::value() const
{
    return static_cast<Brg_GpioValT>(ui->comboBoxValue->currentIndex());
}
