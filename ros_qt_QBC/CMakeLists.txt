##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(ros_qt_demo)
#add
set(CMAKE_INCLUDE_CURRENT_DIR ON)
##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    )
add_message_files(
    FILES
    uav_data.msg
    )
add_service_files(
    FILES
    MissionCommand.srv
    ModeCommand.srv
    PoseCommand.srv
    FormationCommand.srv
    )
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )
include_directories(${catkin_INCLUDE_DIRS})
#add
find_package(Qt5 COMPONENTS Quick REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES demo02_talker_listener
      CATKIN_DEPENDS roscpp message_runtime
    #  DEPENDS system_lib
    )

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#remove
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/ros_qt_demo/*.hpp *.h)

#change
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(ros_qt_demo ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(ros_qt_demo ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS ros_qt_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(Mission_Server Mission_Server.cpp)
add_dependencies(Mission_Server ${PROJECT_NAME}_gencpp)
target_link_libraries(Mission_Server
  ${catkin_LIBRARIES}
)

add_executable(pub_uav_data pub_uav_data.cpp)
add_dependencies(pub_uav_data ${PROJECT_NAME}_gencpp)
target_link_libraries(pub_uav_data
  ${catkin_LIBRARIES}
)

add_executable(translabel translabel.cpp)
add_dependencies(translabel ${PROJECT_NAME}_gencpp)
target_link_libraries(translabel
  ${catkin_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core Qt5::Quick)

