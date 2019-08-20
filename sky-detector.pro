QT += core
QT -= gui

TARGET = sky-detector
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    build/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    build/CMakeFiles/feature_tests.cxx \
    file_processor/file_system_processor.cpp \
    sky_detector/imageSkyDetector.cpp \
    main_test.cpp \
    build/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c \
    build/CMakeFiles/feature_tests.c

HEADERS += \
    file_processor/file_system_processor.h \
    sky_detector/imageSkyDetector.h

DISTFILES += \
    CMakeLists.txt

