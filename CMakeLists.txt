cmake_minimum_required(VERSION 3.5)

project(test_1 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets PrintSupport)

INCLUDE_DIRECTORIES(/usr/xenomai/include /usr/xenomai/include/cobalt /usr/xenomai/include/rtdm /usr/xenomai/include/xenomai
                    /usr/SOEM/soem /usr/SOEM/oshw/linux /usr/SOEM/osal/linux /usr/SOEM/osal
                    /home/mcl/Documents/test_1)
LINK_DIRECTORIES(/usr/xenomai/lib /usr/xenomai/lib/xenomai /usr/SOEM/build)
LINK_LIBRARIES(analogy cobalt copperplate -L/usr/xenomai/lib/xenomai/bootstrap.o rt soem)



#find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
#find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        qcustomplot.cpp
        qcustomplot.h
)


#Find and use the system's Eigen3 Libarary
FIND_PACKAGE(Eigen3 3.3.7)

IF (NOT EIGEN3_FOUND)
    MESSAGE (WARNING "Could not find Eigen3 on your system. Please install it!")
ENDIF (NOT EIGEN3_FOUND)

INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(test_1
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET test_2 APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(test_1 SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(test_1
            ${PROJECT_SOURCES}
            controlwindow.h controlwindow.cpp controlwindow.ui
            data_mutex.h
            data_mutex.cpp
            actuator.h actuator.cpp
            controller.h controller.cpp
            data_logging.h data_logging.cpp
            ecat_func.h
            
            ecat_func.cpp
            filter.h filter.cpp
            kinematics.h kinematics.cpp
            optimization.h optimization.cpp
            trajectory.h trajectory.cpp
            resource.qrc
            
        )
    endif()
endif()



#target_link_libraries(RT_ECAT_MASTER_ELMO_GOLD PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt5::PrintSupport)

target_link_libraries(test_1 PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt5::PrintSupport)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(test_1 PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)


#if(${QT_VERSION} VERSION_LESS 6.1.0)
#  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.test_1)
#endif()
#set_target_properties(test_1 PROPERTIES
#    ${BUNDLE_ID_OPTION}
#    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
#    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
#    MACOSX_BUNDLE TRUE
#    WIN32_EXECUTABLE TRUE
#)

#include(GNUInstallDirs)
#install(TARGETS test_1
#    BUNDLE DESTINATION .
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(test_1)
endif()
