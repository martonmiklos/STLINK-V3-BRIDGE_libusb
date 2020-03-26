#-------------------------------------------------
#
# Project created by QtCreator 2019-07-23T11:17:36
#
#-------------------------------------------------

QT       -= core gui

TARGET = STLINK-V3-BRIDGE
TEMPLATE = lib

DEFINES += STLINKV3BRIDGE_LIBRARY PATH_MAX=4600

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += src/bridge
INCLUDEPATH += src/common

LIBS += -L.
LIBS += -lusb-1.0

SOURCES += \
    src/bridge/bridge.cpp \
    src/common/stlink_interface.cpp \
    src/common/stlink_device.cpp \
    src/common/criticalsectionlock.cpp \
    src/error/ErrLog.cpp

HEADERS += \
    src/bridge/stlink_fw_const_bridge.h \
    src/bridge/stlink_fw_api_bridge.h \
    src/bridge/bridge.h \
    src/common/STLinkUSBDriver.h \
    src/common/stlink_type.h \
    src/common/stlink_interface.h \
    src/common/stlink_if_common.h \
    src/common/stlink_fw_api_common.h \
    src/common/stlink_device.h \
    src/common/criticalsectionlock.h \
    src/error/ErrLog.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

FORMS +=
