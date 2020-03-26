#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "bridge.h"
#include "stlink_interface.h"

#include <QMainWindow>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class GPIOBitSettingsWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_toolButtonOpen_toggled(bool checked);
    void on_toolButtonGPIOInit_clicked();
    void readGPIOs();

private:
    Ui::MainWindow *ui;
    Brg *m_brg = nullptr;
    STLinkInterface m_stInterface;
    QList<GPIOBitSettingsWidget*> m_gpioWidgets;
    QTimer m_pollTimer;
};
#endif // MAINWINDOW_H
