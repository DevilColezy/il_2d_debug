// ===================================================================
//  debug_panel_node.cpp  —  Qt5 Widgets GUI node entry point
//
//  ros::init first (consumes ROS args), then QApplication, then the
//  panel.  ROS spinning happens in the panel's GUI refresh timer
//  (ros::spinOnce), so the Qt event loop stays responsive.
// ===================================================================

#include <QApplication>

#include <ros/ros.h>

#include "il_2d_multiscale_debug/debug_panel.hpp"
#include "il_2d_multiscale_debug/params_io.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "debug_panel_node");
    QApplication app(argc, argv);
    ros::NodeHandle nh, pnh("~");
    const il_2d_multiscale_debug::Params2D p = il_2d_multiscale_debug::loadParams(pnh);
    il_2d_multiscale_debug::DebugPanel panel(p, nh);
    panel.show();
    return app.exec();
}
