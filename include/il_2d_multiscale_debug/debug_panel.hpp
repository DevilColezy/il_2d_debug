#pragma once
/// @file   debug_panel.hpp
/// @brief  Qt5 Widgets debug panel (canvas + controls + status).
///
/// The GUI NEVER holds a pointer to the simulation core.  It interacts
/// with debug_simulation_node ONLY through ROS topics (debug_snapshot,
/// occupancy grids, paths, markers) and services (SetPaused,
/// StepSimulation, ResetTask, GenerateScene, GenerateTask,
/// SetNavigationGoal, SetSimSpeed, ExportFlightLog).

#include <QMainWindow>
#include <QElapsedTimer>
#include <QImage>
#include <QTimer>
#include <QWidget>

#include <functional>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "il_2d_multiscale_debug/DebugSnapshot.h"
#include "il_2d_multiscale_debug/types.hpp"

class QLabel;
class QLineEdit;
class QComboBox;
class QCheckBox;
class QPushButton;
class QTextEdit;
class QSplitter;
class QPainter;
class QPaintEvent;
class QResizeEvent;
class QMouseEvent;

namespace il_2d_multiscale_debug {

/// World-aligned 2D canvas that renders the whole debug state.
class DebugCanvas : public QWidget {
    Q_OBJECT
public:
    explicit DebugCanvas(QWidget* parent = nullptr);

    void setSnapshot(const DebugSnapshot& s) { snap_ = s; update(); }
    void setRegion(double minx, double maxx, double miny, double maxy) {
        minx_ = minx; maxx_ = maxx; miny_ = miny; maxy_ = maxy;
    }
    void setObstacles(const visualization_msgs::MarkerArray& m) { obstacles_ = m; update(); }
    void setEsdf(const nav_msgs::OccupancyGrid::ConstPtr& g) { esdf_ = g; esdfImageDirty_ = true; update(); }
    void setSelectable(const nav_msgs::OccupancyGrid::ConstPtr& g) { selectable_ = g; update(); }
    void setLocalObs(const nav_msgs::OccupancyGrid::ConstPtr& g) { local_obs_ = g; localImageDirty_ = true; update(); }
    /// INSTANTANEOUS FOV patch (raw sensor frame of the current tick) —
    /// drawn as a translucent orange overlay distinct from the merged
    /// history map so the two semantics are visually separated.
    void setPatch(const nav_msgs::OccupancyGrid::ConstPtr& g) { patch_ = g; patchImageDirty_ = true; update(); }
    void setLocalPlan(const nav_msgs::Path::ConstPtr& p) { local_plan_ = p; update(); }
    void setExecutedPath(const nav_msgs::Path::ConstPtr& p) { executed_ = p; update(); }
    void setLeftRoute(const nav_msgs::Path::ConstPtr& p) { left_ = p; update(); }
    void setRightRoute(const nav_msgs::Path::ConstPtr& p) { right_ = p; update(); }
    void setLockedRoute(const nav_msgs::Path::ConstPtr& p) { locked_ = p; update(); }
    void setRejected(const visualization_msgs::MarkerArray::ConstPtr& m) { rejected_ = m; update(); }
    void setDebugMarkers(const visualization_msgs::MarkerArray::ConstPtr& m) { markers_ = m; update(); }

    void setShowEsdf(bool b) { show_esdf_ = b; update(); }
    void setShowTruthPaths(bool b) { show_truth_ = b; update(); }
    void setShowLocalObs(bool b) { show_local_obs_ = b; update(); }
    void setShowRejected(bool b) { show_rejected_ = b; update(); }

    /// World coordinate of a widget pixel (inverse of the view transform).
    bool pixelToWorld(const QPoint& px, double& x, double& y) const;

signals:
    /// Emitted on left-click with the world-space click point.
    void goalClicked(double x, double y);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void rebuildEsdfImage();
    void rebuildLocalImage();
    void rebuildPatchImage();
    void drawOccupancy(QPainter& p, const QImage& img);
    void drawPath(QPainter& p, const nav_msgs::Path::ConstPtr& path,
                  const QColor& color, int width, bool dashed);
    QPointF worldToPixel(double x, double y) const;
    QImage buildGridImage(const nav_msgs::OccupancyGrid::ConstPtr& g,
                          const std::function<int(int8_t)>& color) const;

    // view state
    double minx_ = -20, maxx_ = 20, miny_ = -20, maxy_ = 20;
    double scale_ = 1.0;
    QPointF offset_{0, 0};

    // data
    DebugSnapshot snap_;
    visualization_msgs::MarkerArray obstacles_;
    visualization_msgs::MarkerArray::ConstPtr markers_;
    nav_msgs::OccupancyGrid::ConstPtr esdf_, selectable_, local_obs_, patch_;
    nav_msgs::Path::ConstPtr local_plan_, executed_, left_, right_, locked_;
    visualization_msgs::MarkerArray::ConstPtr rejected_;
    QImage esdf_image_, local_image_, patch_image_;
    bool esdfImageDirty_ = true, localImageDirty_ = true, patchImageDirty_ = true;

    // toggles
    bool show_esdf_ = true;
    bool show_truth_ = true;
    bool show_local_obs_ = true;
    bool show_rejected_ = true;
};

/// Main debug panel window.
class DebugPanel : public QMainWindow {
    Q_OBJECT
public:
    DebugPanel(const Params2D& p, ros::NodeHandle& nh);
    ~DebugPanel() override;

private slots:
    void onRun();
    void onPause();
    void onSingleStep();
    void onStepTo5Hz();
    void onResetTask();
    void onNewTask();
    void onNewScene();
    void onExportLog();
    void onSpeedChanged(int idx);
    void onToggleChanged();
    void onTimer();
    void onGoalClicked(double x, double y);

private:
    void requestPause(bool paused);
    void callStep(uint32_t steps, bool to_5hz);
    void callSetSpeed(double speed);
    void refreshStatus();
    uint64_t parseSeed() const;
    std::string lastServiceInfo();
    /// Call a service and record success ONLY when the ROS call succeeded
    /// AND response.success is true (failure reason kept in
    /// last_srv_result_).  Defined inline so all slots can use it.
    template <typename Srv>
    bool callService(ros::ServiceClient& cli, Srv& srv) {
        const bool called = cli.call(srv);
        last_srv_ok_ = called && srv.response.success;
        last_srv_result_ = called ? srv.response.reason : "service unavailable";
        return last_srv_ok_;
    }

    Params2D p_;
    ros::NodeHandle& nh_;

    DebugCanvas* canvas_ = nullptr;
    QPushButton* run_btn_ = nullptr;
    QPushButton* pause_btn_ = nullptr;
    QLineEdit* seed_edit_ = nullptr;
    QComboBox* speed_combo_ = nullptr;
    QCheckBox* cb_esdf_ = nullptr;
    QCheckBox* cb_truth_ = nullptr;
    QCheckBox* cb_local_ = nullptr;
    QCheckBox* cb_rejected_ = nullptr;
    QLabel* paused_label_ = nullptr;
    QTextEdit* status_ = nullptr;

    QTimer timer_;
    ros::Subscriber sub_snapshot_, sub_obs_, sub_patch_, sub_esdf_, sub_selectable_,
        sub_local_plan_, sub_executed_, sub_left_, sub_right_, sub_locked_,
        sub_rejected_, sub_obstacles_, sub_markers_;
    ros::ServiceClient cli_pause_, cli_step_, cli_reset_, cli_new_scene_,
        cli_new_task_, cli_goal_, cli_speed_, cli_export_log_;
    DebugSnapshot latest_snapshot_;
    std::string last_srv_result_;
    bool last_srv_ok_ = false;

    // ── ROS topic callbacks ────────────────────────────────────────
    void snapshotCb(const DebugSnapshot::ConstPtr& m);
    void obsCb(const nav_msgs::OccupancyGrid::ConstPtr& m);
    void patchCb(const nav_msgs::OccupancyGrid::ConstPtr& m);
    void esdfCb(const nav_msgs::OccupancyGrid::ConstPtr& m);
    void selectableCb(const nav_msgs::OccupancyGrid::ConstPtr& m);
    void localPlanCb(const nav_msgs::Path::ConstPtr& m);
    void executedCb(const nav_msgs::Path::ConstPtr& m);
    void leftCb(const nav_msgs::Path::ConstPtr& m);
    void rightCb(const nav_msgs::Path::ConstPtr& m);
    void lockedCb(const nav_msgs::Path::ConstPtr& m);
    void rejectedCb(const visualization_msgs::MarkerArray::ConstPtr& m);
    void markersCb(const visualization_msgs::MarkerArray::ConstPtr& m);
    void obstaclesCb(const visualization_msgs::MarkerArray& m);
};

}  // namespace il_2d_multiscale_debug
