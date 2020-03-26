#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_stInterface.LoadStlinkLibrary(""); // libusb implementation does not need param

    ui->widgetGPIO0->setIndex(0);
    ui->widgetGPIO1->setIndex(1);
    ui->widgetGPIO2->setIndex(2);
    ui->widgetGPIO3->setIndex(3);

    m_gpioWidgets << ui->widgetGPIO0 << ui->widgetGPIO1 << ui->widgetGPIO2 << ui->widgetGPIO3;

    m_brg = new Brg(m_stInterface);

    uint32_t devCnt = 0;
    m_stInterface.EnumDevices(&devCnt, true);
    for (uint32_t i = 0; i<devCnt; i++) {
        STLink_DeviceInfo2T devInfo;
        if (m_stInterface.GetDeviceInfo2((int)i, &devInfo, 0) == STLINKIF_NO_ERR) {
            ui->comboBoxDevice->addItem(devInfo.EnumUniqueId);
        }
    }

    m_pollTimer.setSingleShot(false);
    m_pollTimer.setInterval(100);
    connect(&m_pollTimer, &QTimer::timeout, this, &MainWindow::readGPIOs);
}

MainWindow::~MainWindow()
{
    m_brg->CloseStlink();
    delete ui;
}

void MainWindow::on_toolButtonOpen_toggled(bool checked)
{
    if (checked) {
        Brg_StatusT ret = m_brg->OpenStlink(ui->comboBoxDevice->currentText().toUtf8().constData(), true);
        if (ret == BRG_NO_ERR) {
            ui->statusbar->showMessage(tr("Device opened successfully"));
            ui->toolButtonGPIOInit->setEnabled(true);
            on_toolButtonGPIOInit_clicked();
            ui->toolButtonOpen->setText(tr("Close"));
        } else {
            ui->toolButtonOpen->setChecked(false);
            ui->statusbar->showMessage(tr("Device open failed with error: %1").arg(ret));
        }
    } else {
        ui->toolButtonOpen->setText(tr("Open"));
        m_brg->CloseStlink();
        ui->toolButtonGPIOInit->setEnabled(false);
        m_pollTimer.stop();
    }
}

void MainWindow::on_toolButtonGPIOInit_clicked()
{
    Brg_GpioConfT confs[4];
    for (auto widget : m_gpioWidgets) {
        confs[widget->index()].Mode = widget->mode();
        confs[widget->index()].Pull = widget->pull();
        confs[widget->index()].Speed = widget->speed();
        confs[widget->index()].OutputType = widget->outputType();
    }
    Brg_GpioInitT init;
    init.ConfigNb = 4;
    init.GpioMask = BRG_GPIO_ALL;
    init.pGpioConf = confs;
    auto ret = m_brg->InitGPIO(&init);
    if (ret != BRG_NO_ERR){
        ui->statusbar->showMessage(tr("GPIO init failed with error: %1").arg(ret));
    }
    m_pollTimer.start();
}

void MainWindow::readGPIOs()
{
    Brg_GpioValT inputLevels[4];
    uint8_t errorMask;
    auto ret = m_brg->ReadGPIO(BRG_GPIO_ALL, inputLevels, &errorMask);
    if (ret != BRG_NO_ERR){
        ui->statusbar->showMessage(tr("GPIO read failed with error: %1").arg(ret));
        m_pollTimer.stop();
    }

    uint8_t mask = 0;
    Brg_GpioValT outLevels[4];
    for (int i = 0; i<4; i++) {
        if (m_gpioWidgets[i]->mode() == GPIO_MODE_INPUT)
            m_gpioWidgets[i]->setValue(inputLevels[i]);
        else {
            mask |= (1<<i);
            outLevels[i] = m_gpioWidgets[i]->value();
        }
    }

    if (mask) {
        ret = m_brg->SetResetGPIO(mask, outLevels, &errorMask);
        if (ret != BRG_NO_ERR){
            ui->statusbar->showMessage(tr("GPIO write failed with error: %1").arg(ret));
            m_pollTimer.stop();
        }
    }
}
