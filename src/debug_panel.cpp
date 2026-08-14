// ===================================================================
//  debug_panel.cpp  —  Qt5 Widgets interactive debug panel
//
//  Canvas rendering of the full 2D debug state, control buttons, seed /
//  speed controls, display toggles and the status/audit panel.
//
//  INFORMATION BOUNDARY: the GUI talks to debug_simulation_node ONLY
//  through ROS topics and services.  It holds no pointer to the
//  simulation core and cannot mutate sim state directly.
// ===================================================================

#include "il_2d_multiscale_debug/debug_panel.hpp"

#include <QtWidgets>
#include <QPainterPath>
#include <QFontDatabase>

#include "il_2d_multiscale_debug/GenerateScene.h"
#include "il_2d_multiscale_debug/GenerateTask.h"
#include "il_2d_multiscale_debug/ExportFlightLog.h"
#include "il_2d_multiscale_debug/ResetTask.h"
#include "il_2d_multiscale_debug/SetNavigationGoal.h"
#include "il_2d_multiscale_debug/SetPaused.h"
#include "il_2d_multiscale_debug/SetSimSpeed.h"
#include "il_2d_multiscale_debug/StepSimulation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace il_2d_multiscale_debug {

// ═══════════════════════════════════════════════════════════════════
//  Colors (documented in the README)
// ═══════════════════════════════════════════════════════════════════
namespace {
QRgb esdfColor(int8_t v) {
    const double t = clamp(static_cast<double>(v), 0.0, 100.0) / 100.0;
    const int r = static_cast<int>((0.27 + 0.72 * t) * 255.0);
    const int g = static_cast<int>((0.00 + 0.91 * t) * 255.0);
    const int b = static_cast<int>((0.33 - 0.19 * t) * 255.0);
    return qRgb(r, g, b);
}
QRgb localColor(int8_t v) {
    if (v < 0) return qRgb(60, 60, 62);       // UNKNOWN
    if (v >= 100) return qRgb(210, 30, 30);   // OCCUPIED
    return qRgb(30, 160, 70);                 // FREE
}
// INSTANTANEOUS FOV patch — orange tint so it is clearly distinct from
// the merged history map (which uses green/red/grey).
QRgb patchColor(int8_t v) {
    if (v < 0) return qRgb(0, 0, 0);           // UNKNOWN → transparent-ish
    if (v >= 100) return qRgb(255, 140, 0);    // OCCUPIED → orange
    return qRgb(255, 220, 150);                // FREE → pale orange
}
QRgb selectableColor(int8_t v) {
    return (v >= 100) ? qRgb(255, 255, 255) : qRgb(0, 0, 0);
}
}  // namespace

// ═══════════════════════════════════════════════════════════════════
//  DebugCanvas
// ═══════════════════════════════════════════════════════════════════
DebugCanvas::DebugCanvas(QWidget* parent) : QWidget(parent) {
    setMinimumSize(640, 640);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void DebugCanvas::resizeEvent(QResizeEvent*) {
    const double rw = maxx_ - minx_, rh = maxy_ - miny_;
    if (rw <= 0.0 || rh <= 0.0) return;
    scale_ = std::min(static_cast<double>(width()) / rw,
                      static_cast<double>(height()) / rh);
    offset_ = QPointF((width() - rw * scale_) / 2.0, (height() - rh * scale_) / 2.0);
}

QPointF DebugCanvas::worldToPixel(double x, double y) const {
    return QPointF(offset_.x() + (x - minx_) * scale_,
                   offset_.y() + (maxy_ - y) * scale_);
}

bool DebugCanvas::pixelToWorld(const QPoint& px, double& x, double& y) const {
    x = minx_ + (px.x() - offset_.x()) / scale_;
    y = maxy_ - (px.y() - offset_.y()) / scale_;
    return x >= minx_ && x <= maxx_ && y >= miny_ && y <= maxy_;
}

void DebugCanvas::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        double x, y;
        if (pixelToWorld(event->pos(), x, y)) emit goalClicked(x, y);
    }
    QWidget::mousePressEvent(event);
}

QImage DebugCanvas::buildGridImage(const nav_msgs::OccupancyGrid::ConstPtr& g,
                                   const std::function<int(int8_t)>& color) const {
    QImage img(g->info.width, g->info.height, QImage::Format_RGB32);
    for (uint32_t iy = 0; iy < g->info.height; ++iy) {
        for (uint32_t ix = 0; ix < g->info.width; ++ix) {
            const int8_t v = g->data[static_cast<size_t>(iy) * g->info.width + ix];
            // Row 0 of the image = highest world Y (flip rows).
            img.setPixel(ix, g->info.height - 1 - iy, color(v));
        }
    }
    return img;
}

void DebugCanvas::rebuildEsdfImage() {
    if (esdf_ && esdf_->info.width > 0) {
        esdf_image_ = buildGridImage(esdf_, esdfColor);
        esdfImageDirty_ = false;
    }
}

void DebugCanvas::rebuildLocalImage() {
    if (local_obs_ && local_obs_->info.width > 0) {
        local_image_ = buildGridImage(local_obs_, localColor);
        localImageDirty_ = false;
    }
}

void DebugCanvas::rebuildPatchImage() {
    if (patch_ && patch_->info.width > 0) {
        patch_image_ = buildGridImage(patch_, patchColor);
        patchImageDirty_ = false;
    }
}

void DebugCanvas::drawOccupancy(QPainter& p, const QImage& img) {
    if (!esdf_ || esdf_->info.width == 0) return;
    const double x0 = esdf_->info.origin.position.x;
    const double y0 = esdf_->info.origin.position.y;
    const double x1 = x0 + esdf_->info.width * esdf_->info.resolution;
    const double y1 = y0 + esdf_->info.height * esdf_->info.resolution;
    const QPointF tl = worldToPixel(x0, y1);
    const QPointF br = worldToPixel(x1, y0);
    p.drawImage(QRectF(tl, br), img);
}

void DebugCanvas::drawPath(QPainter& p, const nav_msgs::Path::ConstPtr& path,
                           const QColor& color, int width, bool dashed) {
    if (!path || path->poses.empty()) return;
    QPen pen(color, width);
    if (dashed) {
        pen.setStyle(Qt::DashLine);
        pen.setDashPattern({4.0, 4.0});
    }
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    QPainterPath qp;
    bool first = true;
    for (const auto& ps : path->poses) {
        const QPointF pt = worldToPixel(ps.pose.position.x, ps.pose.position.y);
        if (first) {
            qp.moveTo(pt);
            first = false;
        } else {
            qp.lineTo(pt);
        }
    }
    p.drawPath(qp);
}

void DebugCanvas::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(22, 22, 26));
    p.setRenderHint(QPainter::Antialiasing, true);

    // 1) Truth ESDF heatmap (privileged display only).
    if (show_esdf_ && esdf_ && esdf_->info.width > 0) {
        if (esdfImageDirty_) rebuildEsdfImage();
        p.setOpacity(0.45);
        drawOccupancy(p, esdf_image_);
        p.setOpacity(1.0);
    }
    // 2) Selectable mask (faint overlay).
    if (show_esdf_ && selectable_ && selectable_->info.width > 0) {
        const QImage sel = buildGridImage(selectable_, selectableColor);
        const double x0 = selectable_->info.origin.position.x;
        const double y0 = selectable_->info.origin.position.y;
        const double x1 = x0 + selectable_->info.width * selectable_->info.resolution;
        const double y1 = y0 + selectable_->info.height * selectable_->info.resolution;
        p.setOpacity(0.12);
        p.drawImage(QRectF(worldToPixel(x0, y1), worldToPixel(x1, y0)), sel);
        p.setOpacity(1.0);
    }
    // 3) Local observation (FREE / OCCUPIED / UNKNOWN).
    if (show_local_obs_ && local_obs_ && local_obs_->info.width > 0) {
        if (localImageDirty_) rebuildLocalImage();
        const double x0 = local_obs_->info.origin.position.x;
        const double y0 = local_obs_->info.origin.position.y;
        const double x1 = x0 + local_obs_->info.width * local_obs_->info.resolution;
        const double y1 = y0 + local_obs_->info.height * local_obs_->info.resolution;
        p.setOpacity(0.5);
        p.drawImage(QRectF(worldToPixel(x0, y1), worldToPixel(x1, y0)), local_image_);
        p.setOpacity(1.0);
    }

    // 3.5) INSTANTANEOUS FOV patch overlay (orange) — the raw sensor
    //      frame of the current tick, visually distinct from the merged
    //      history map above.  OCCUPIED cells are the strong orange ring.
    if (show_local_obs_ && patch_ && patch_->info.width > 0) {
        if (patchImageDirty_) rebuildPatchImage();
        const double x0 = patch_->info.origin.position.x;
        const double y0 = patch_->info.origin.position.y;
        const double x1 = x0 + patch_->info.width * patch_->info.resolution;
        const double y1 = y0 + patch_->info.height * patch_->info.resolution;
        p.setOpacity(0.4);
        p.drawImage(QRectF(worldToPixel(x0, y1), worldToPixel(x1, y0)), patch_image_);
        p.setOpacity(1.0);
        QFont f = QFontDatabase::systemFont(QFontDatabase::FixedFont);
        f.setPixelSize(10);
        p.setFont(f);
        p.setPen(QColor(255, 150, 40));
        p.drawText(QPointF(worldToPixel(x0, y1).x() + 4, worldToPixel(x0, y1).y() - 4),
                   "INSTANT PATCH (orange) / HISTORY MAP (green·red·grey)");
    }

    // 4) Region boundary + obstacles + start/goal + pending goal are all
    // privileged scene truth and follow the truth-display toggle.
    int task_marker_count = 0;  // start/goal identified by order, not id parity
    if (show_truth_) for (const auto& mk : obstacles_.markers) {
        if (mk.action != visualization_msgs::Marker::ADD) continue;  // skip DELETE*/
        const QPointF c = worldToPixel(mk.pose.position.x, mk.pose.position.y);
        if (mk.ns == "region" && mk.type == visualization_msgs::Marker::LINE_STRIP) {
            QPen pen(QColor(200, 200, 200), 2);
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            QPainterPath qp;
            bool first = true;
            for (const auto& pt : mk.points) {
                const QPointF wp = worldToPixel(pt.x, pt.y);
                if (first) {
                    qp.moveTo(wp);
                    first = false;
                } else {
                    qp.lineTo(wp);
                }
            }
            p.drawPath(qp);
        } else if (mk.ns == "obstacle" &&
                   mk.type == visualization_msgs::Marker::CYLINDER) {
            const double r = mk.scale.x / 2.0 * scale_;
            p.setBrush(QColor(150, 25, 25, 220));
            p.setPen(QPen(QColor(230, 70, 70), 1.5));
            p.drawEllipse(c, r, r);
        } else if (mk.ns == "task" && mk.type == visualization_msgs::Marker::SPHERE) {
            const double r = mk.scale.x / 2.0 * scale_;
            const bool is_start = (task_marker_count == 0);
            ++task_marker_count;
            p.setBrush(is_start ? QColor(40, 190, 60) : QColor(240, 200, 40));
            p.setPen(QPen(QColor(255, 255, 255), 1.5));
            p.drawEllipse(c, r, r);
        } else if (mk.ns == "pending_goal" &&
                   mk.type == visualization_msgs::Marker::SPHERE) {
            // PENDING (not yet accepted) final goal — translucent, dashed.
            const double r = mk.scale.x / 2.0 * scale_;
            p.setBrush(QColor(255, 150, 0, 60));
            p.setPen(QPen(QColor(255, 170, 40), 2, Qt::DashLine));
            p.drawEllipse(c, r, r);
        } else if (mk.ns == "pending_goal" &&
                   mk.type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
            QFont f = QFontDatabase::systemFont(QFontDatabase::FixedFont);
            f.setPixelSize(10);
            p.setFont(f);
            p.setPen(QColor(255, 170, 40));
            p.drawText(QPointF(c.x() - 40, c.y()), QString::fromStdString(mk.text));
        }
    }

    // 5) Rejected candidates (dim gray).
    if (show_rejected_ && rejected_) {
        p.setOpacity(0.55);
        for (const auto& mk : rejected_->markers) {
            if (mk.action != visualization_msgs::Marker::ADD) continue;
            QPen pen(QColor(120, 120, 120), 1);
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            QPainterPath qp;
            bool first = true;
            for (const auto& pt : mk.points) {
                const QPointF wp = worldToPixel(pt.x, pt.y);
                if (first) {
                    qp.moveTo(wp);
                    first = false;
                } else {
                    qp.lineTo(wp);
                }
            }
            p.drawPath(qp);
        }
        p.setOpacity(1.0);
    }

    // 6) Privileged routes (truth paths) + local plan + executed path.
    if (show_truth_) {
        drawPath(p, left_, QColor(0, 210, 60), 2, true);
        drawPath(p, right_, QColor(230, 40, 40), 2, true);
    }
    if (show_truth_) {
        drawPath(p, locked_, QColor(255, 200, 30), 3, false);
    }
    drawPath(p, local_plan_, QColor(40, 140, 255), 2, false);
    drawPath(p, executed_, QColor(0, 230, 230), 1, false);

    // 7) Dynamic debug markers (vehicle, FOV, target, blocker, velocity).
    if (markers_) {
        for (const auto& mk : markers_->markers) {
            if (mk.action != visualization_msgs::Marker::ADD) continue;  // skip DELETE*
            const QPointF c = worldToPixel(mk.pose.position.x, mk.pose.position.y);
            if (mk.ns == "vehicle" && mk.type == visualization_msgs::Marker::TRIANGLE_LIST) {
                QPolygonF poly;
                for (const auto& pt : mk.points) poly << worldToPixel(pt.x, pt.y);
                p.setBrush(QColor(255, 230, 60));
                p.setPen(QPen(QColor(255, 255, 255), 1));
                p.drawPolygon(poly);
            } else if (mk.ns == "fov" &&
                       mk.type == visualization_msgs::Marker::TRIANGLE_LIST) {
                // FOV fan is published as a TRIANGLE_LIST (groups of 3
                // points: centre, arc_i, arc_{i+1}); draw each triangle.
                for (size_t tri = 0; tri + 2 < mk.points.size(); tri += 3) {
                    QPolygonF poly;
                    poly << worldToPixel(mk.points[tri].x, mk.points[tri].y)
                         << worldToPixel(mk.points[tri + 1].x, mk.points[tri + 1].y)
                         << worldToPixel(mk.points[tri + 2].x, mk.points[tri + 2].y);
                    p.setBrush(QColor(255, 255, 255, 38));
                    p.setPen(QPen(QColor(255, 255, 255, 120), 1));
                    p.drawPolygon(poly);
                }
            } else if (mk.ns == "local_target" &&
                       mk.type == visualization_msgs::Marker::SPHERE) {
                const double r = mk.scale.x / 2.0 * scale_;
                p.setBrush(Qt::NoBrush);
                p.setPen(QPen(QColor(0, 240, 240), 2));
                p.drawEllipse(c, r, r);
            } else if (mk.ns == "blocker" &&
                       mk.type == visualization_msgs::Marker::SPHERE) {
                const double r = mk.scale.x / 2.0 * scale_;
                p.setBrush(QColor(255, 130, 0, 70));
                p.setPen(QPen(QColor(255, 150, 0), 2, Qt::DashLine));
                p.drawEllipse(c, r, r);
            } else if (mk.ns == "velocity" &&
                       mk.type == visualization_msgs::Marker::ARROW) {
                if (mk.points.size() >= 2) {
                    const QPointF start =
                        worldToPixel(mk.points[0].x, mk.points[0].y);
                    const QPointF tip =
                        worldToPixel(mk.points[1].x, mk.points[1].y);
                    QPen pen(QColor(255, 0, 255), 2.5);
                    p.setPen(pen);
                    p.setBrush(QColor(255, 0, 255));
                    QLineF line(start, tip);
                    p.drawLine(line);
                    const double ang = std::atan2(line.dy(), line.dx());
                    const double hl = std::min(8.0, line.length() * 0.3);
                    QPointF a1(tip.x() - hl * std::cos(ang - 0.5),
                               tip.y() - hl * std::sin(ang - 0.5));
                    QPointF a2(tip.x() - hl * std::cos(ang + 0.5),
                               tip.y() - hl * std::sin(ang + 0.5));
                    p.drawPolygon(QPolygonF({tip, a1, a2}));
                }
            } else if (mk.type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
                QFont f = QFontDatabase::systemFont(QFontDatabase::FixedFont);
                f.setPixelSize(std::max(9, static_cast<int>(mk.scale.z * 9.0)));
                p.setFont(f);
                p.setPen(QColor(static_cast<int>(mk.color.r * 255),
                                static_cast<int>(mk.color.g * 255),
                                static_cast<int>(mk.color.b * 255)));
                p.drawText(QPointF(c.x() - 40, c.y()), QString::fromStdString(mk.text));
            }
        }
    }

    // 8) Legend hint.
    p.setPen(QColor(180, 180, 180));
    p.setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    p.drawText(QRect(6, 6, width() - 12, 40),
               "Click on free space to set a new navigation goal");
}

// ═══════════════════════════════════════════════════════════════════
//  DebugPanel
// ═══════════════════════════════════════════════════════════════════
DebugPanel::DebugPanel(const Params2D& p, ros::NodeHandle& nh) : p_(p), nh_(nh) {
    setWindowTitle("il_2d_multiscale_debug — 2D Hierarchical Expert Panel");

    canvas_ = new DebugCanvas(this);
    canvas_->setRegion(p_.region_min_x, p_.region_max_x, p_.region_min_y,
                       p_.region_max_y);
    canvas_->setShowEsdf(p_.gui_show_esdf);
    canvas_->setShowTruthPaths(p_.gui_show_truth_paths);
    canvas_->setShowLocalObs(p_.gui_show_local_observation);
    canvas_->setShowRejected(p_.gui_show_rejected_candidates);

    // ── Control / status side panel ────────────────────────────────
    QWidget* side = new QWidget(this);
    QVBoxLayout* lay = new QVBoxLayout(side);

    run_btn_ = new QPushButton("Run", side);
    pause_btn_ = new QPushButton("Pause", side);
    QPushButton* step_btn = new QPushButton("Single 30Hz Step", side);
    QPushButton* step5_btn = new QPushButton("Step To Next 5Hz Tick", side);
    QPushButton* reset_btn = new QPushButton("Reset Current Task", side);
    QPushButton* newtask_btn = new QPushButton("New Task In Same Scene", side);
    QPushButton* newscene_btn = new QPushButton("New Scene", side);
    QPushButton* export_btn = new QPushButton("Export Current Task Log", side);

    // Initial pause label comes from the launch `paused` arg (private
    // namespace), so `paused:=false` shows RUNNING immediately without
    // waiting for the first snapshot.  After that the label is ALWAYS
    // driven by the authoritative snapshot.paused field.
    bool init_paused = true;
    ros::NodeHandle pnh("~");
    pnh.param("paused", init_paused, true);
    paused_label_ = new QLabel(init_paused ? "STATE: PAUSED" : "STATE: RUNNING", side);
    paused_label_->setStyleSheet("color:#ffcc00;font-weight:bold;");
    lay->addWidget(paused_label_);
    lay->addWidget(run_btn_);
    lay->addWidget(pause_btn_);
    lay->addWidget(step_btn);
    lay->addWidget(step5_btn);

    lay->addSpacing(8);
    lay->addWidget(new QLabel("seed:", side));
    seed_edit_ = new QLineEdit(side);
    seed_edit_->setText(QString::number(static_cast<unsigned long long>(p_.default_seed)));
    lay->addWidget(seed_edit_);

    lay->addWidget(new QLabel("simulation speed:", side));
    speed_combo_ = new QComboBox(side);
    for (double s : p_.gui_speeds) {
        speed_combo_->addItem(QString::number(s));
    }
    const int def_idx = std::max(0, speed_combo_->findText(
                                        QString::number(p_.gui_default_speed)));
    speed_combo_->setCurrentIndex(def_idx);
    lay->addWidget(speed_combo_);

    lay->addSpacing(8);
    lay->addWidget(new QLabel("display:", side));
    cb_esdf_ = new QCheckBox("ESDF (privileged)", side);
    cb_esdf_->setChecked(p_.gui_show_esdf);
    cb_truth_ = new QCheckBox("truth privileged paths", side);
    cb_truth_->setChecked(p_.gui_show_truth_paths);
    cb_local_ = new QCheckBox("local observation", side);
    cb_local_->setChecked(p_.gui_show_local_observation);
    cb_rejected_ = new QCheckBox("rejected candidates", side);
    cb_rejected_->setChecked(p_.gui_show_rejected_candidates);
    lay->addWidget(cb_esdf_);
    lay->addWidget(cb_truth_);
    lay->addWidget(cb_local_);
    lay->addWidget(cb_rejected_);

    lay->addSpacing(8);
    lay->addWidget(new QLabel("status / audit:", side));
    status_ = new QTextEdit(side);
    status_->setReadOnly(true);
    status_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    status_->setMinimumHeight(320);
    lay->addWidget(status_, 1);

    lay->addWidget(reset_btn);
    lay->addWidget(newtask_btn);
    lay->addWidget(newscene_btn);
    lay->addWidget(export_btn);

    QSplitter* split = new QSplitter(this);
    split->addWidget(canvas_);
    split->addWidget(side);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 1);
    setCentralWidget(split);
    resize(1280, 800);

    // ── Signals ────────────────────────────────────────────────────
    connect(run_btn_, &QPushButton::clicked, this, &DebugPanel::onRun);
    connect(pause_btn_, &QPushButton::clicked, this, &DebugPanel::onPause);
    connect(step_btn, &QPushButton::clicked, this, &DebugPanel::onSingleStep);
    connect(step5_btn, &QPushButton::clicked, this, &DebugPanel::onStepTo5Hz);
    connect(reset_btn, &QPushButton::clicked, this, &DebugPanel::onResetTask);
    connect(newtask_btn, &QPushButton::clicked, this, &DebugPanel::onNewTask);
    connect(newscene_btn, &QPushButton::clicked, this, &DebugPanel::onNewScene);
    connect(export_btn, &QPushButton::clicked, this, &DebugPanel::onExportLog);
    connect(speed_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &DebugPanel::onSpeedChanged);
    connect(cb_esdf_, &QCheckBox::toggled, this, &DebugPanel::onToggleChanged);
    connect(cb_truth_, &QCheckBox::toggled, this, &DebugPanel::onToggleChanged);
    connect(cb_local_, &QCheckBox::toggled, this, &DebugPanel::onToggleChanged);
    connect(cb_rejected_, &QCheckBox::toggled, this, &DebugPanel::onToggleChanged);
    connect(canvas_, &DebugCanvas::goalClicked, this, &DebugPanel::onGoalClicked);

    // ── ROS subscriptions (GUI consumes ONLY published data) ───────
    sub_snapshot_ = nh_.subscribe("debug_snapshot", 10, &DebugPanel::snapshotCb, this);
    sub_obs_ = nh_.subscribe("local_observation", 2, &DebugPanel::obsCb, this);
    sub_patch_ = nh_.subscribe("instantaneous_patch", 2, &DebugPanel::patchCb, this);
    sub_esdf_ = nh_.subscribe("truth_esdf", 1, &DebugPanel::esdfCb, this);
    sub_selectable_ = nh_.subscribe("selectable_mask", 1, &DebugPanel::selectableCb, this);
    sub_local_plan_ = nh_.subscribe("local_plan", 2, &DebugPanel::localPlanCb, this);
    sub_executed_ = nh_.subscribe("executed_path", 2, &DebugPanel::executedCb, this);
    sub_left_ = nh_.subscribe("left_privileged_path", 1, &DebugPanel::leftCb, this);
    sub_right_ = nh_.subscribe("right_privileged_path", 1, &DebugPanel::rightCb, this);
    sub_locked_ = nh_.subscribe("locked_route", 1, &DebugPanel::lockedCb, this);
    sub_rejected_ = nh_.subscribe("rejected_candidates", 2, &DebugPanel::rejectedCb, this);
    sub_markers_ = nh_.subscribe("debug_markers", 2, &DebugPanel::markersCb, this);
    sub_obstacles_ = nh_.subscribe("scene_obstacles", 1, &DebugPanel::obstaclesCb, this);

    // ── ROS service clients ────────────────────────────────────────
    cli_pause_ = nh_.serviceClient<SetPaused>("set_paused");
    cli_step_ = nh_.serviceClient<StepSimulation>("step_simulation");
    cli_reset_ = nh_.serviceClient<ResetTask>("reset_task");
    cli_new_scene_ = nh_.serviceClient<GenerateScene>("generate_scene");
    cli_new_task_ = nh_.serviceClient<GenerateTask>("generate_task");
    cli_goal_ = nh_.serviceClient<SetNavigationGoal>("set_navigation_goal");
    cli_speed_ = nh_.serviceClient<SetSimSpeed>("set_sim_speed");
    cli_export_log_ = nh_.serviceClient<ExportFlightLog>("export_flight_log");

    // ── GUI refresh timer ──────────────────────────────────────────
    const double hz = std::max(1.0, p_.gui_refresh_rate_hz);
    timer_.setInterval(static_cast<int>(1000.0 / hz));
    connect(&timer_, &QTimer::timeout, this, &DebugPanel::onTimer);
    timer_.start();

    refreshStatus();
}

DebugPanel::~DebugPanel() = default;

// ── Callbacks ─────────────────────────────────────────────────────
void DebugPanel::snapshotCb(const DebugSnapshot::ConstPtr& m) {
    latest_snapshot_ = *m;
    canvas_->setSnapshot(*m);
    // Authoritative pause label: the node publishes the real state, so the
    // label always follows the snapshot (never the last request value).
    paused_label_->setText(m->paused ? "STATE: PAUSED" : "STATE: RUNNING");
}
void DebugPanel::obsCb(const nav_msgs::OccupancyGrid::ConstPtr& m) { canvas_->setLocalObs(m); }
void DebugPanel::patchCb(const nav_msgs::OccupancyGrid::ConstPtr& m) { canvas_->setPatch(m); }
void DebugPanel::esdfCb(const nav_msgs::OccupancyGrid::ConstPtr& m) { canvas_->setEsdf(m); }
void DebugPanel::selectableCb(const nav_msgs::OccupancyGrid::ConstPtr& m) { canvas_->setSelectable(m); }
void DebugPanel::localPlanCb(const nav_msgs::Path::ConstPtr& m) { canvas_->setLocalPlan(m); }
void DebugPanel::executedCb(const nav_msgs::Path::ConstPtr& m) { canvas_->setExecutedPath(m); }
void DebugPanel::leftCb(const nav_msgs::Path::ConstPtr& m) { canvas_->setLeftRoute(m); }
void DebugPanel::rightCb(const nav_msgs::Path::ConstPtr& m) { canvas_->setRightRoute(m); }
void DebugPanel::lockedCb(const nav_msgs::Path::ConstPtr& m) { canvas_->setLockedRoute(m); }
void DebugPanel::rejectedCb(const visualization_msgs::MarkerArray::ConstPtr& m) { canvas_->setRejected(m); }
void DebugPanel::markersCb(const visualization_msgs::MarkerArray::ConstPtr& m) { canvas_->setDebugMarkers(m); }
void DebugPanel::obstaclesCb(const visualization_msgs::MarkerArray& m) { canvas_->setObstacles(m); }

// ── Slots ─────────────────────────────────────────────────────────
void DebugPanel::onRun() { requestPause(false); }
void DebugPanel::onPause() { requestPause(true); }

void DebugPanel::requestPause(bool paused) {
    SetPaused srv;
    srv.request.paused = paused;
    if (!callService(cli_pause_, srv)) {
        // Service call or response.success failed → do NOT write the
        // requested value into the label; wait for the authoritative
        // snapshot (the node is the single source of the paused state).
        refreshStatus();
        return;
    }
    // Success: the node now holds the new state; the snapshot will confirm
    // it, but the label may update optimistically right away.
    paused_label_->setText(paused ? "STATE: PAUSED" : "STATE: RUNNING");
    refreshStatus();
}

void DebugPanel::callStep(uint32_t steps, bool to_5hz) {
    StepSimulation srv;
    srv.request.steps = steps;
    srv.request.step_to_next_5hz = to_5hz;
    if (!callService(cli_step_, srv)) {
        // Failure (e.g. terminal — nothing advanced): do NOT guess the
        // paused state; wait for the authoritative snapshot.
        refreshStatus();
        return;
    }
    // Success: the step services are ATOMIC pause operations — the
    // simulation is guaranteed paused when they return, so PAUSED here is
    // authoritative (matching the next snapshot).
    paused_label_->setText("STATE: PAUSED");
    refreshStatus();
}

void DebugPanel::onSingleStep() { callStep(1, false); }
void DebugPanel::onStepTo5Hz() { callStep(0, true); }

void DebugPanel::onResetTask() {
    // Reset restarts the CURRENT, IDENTICAL task (same scene/start/goal/
    // yaw/task_id).  The seed fields below are interface compatibility
    // only — the node ignores them for reset (see ResetTask.srv).
    ResetTask srv;
    srv.request.seed = 0;
    srv.request.use_default_seed = true;
    callService(cli_reset_, srv);
    refreshStatus();
}

void DebugPanel::onNewTask() {
    GenerateTask srv;
    srv.request.seed = 0;
    srv.request.use_default_seed = true;  // derive a task seed from the scene seed
    callService(cli_new_task_, srv);
    refreshStatus();
}

void DebugPanel::onNewScene() {
    GenerateScene srv;
    // The scene generator is deterministic by seed.  The launch starts with
    // the seed shown in the edit box, so submitting that unchanged value
    // would faithfully recreate the same obstacles and make the button look
    // ineffective.  Treat an unchanged value as "next scene" and advance it
    // once; users can still enter any explicit seed to reproduce a scene.
    uint64_t requested_seed = parseSeed();
    if (latest_snapshot_.simulation_initialized &&
        requested_seed == latest_snapshot_.seed) {
        requested_seed =
            requested_seed == std::numeric_limits<uint64_t>::max()
                ? 0
                : requested_seed + 1;
    }
    srv.request.seed = requested_seed;
    srv.request.use_default_seed = false;  // the seed field is authoritative (0 legal)
    if (callService(cli_new_scene_, srv)) {
        // Display the next seed so consecutive button presses keep producing
        // distinct deterministic scenes without extra user input.
        const uint64_t next_seed =
            requested_seed == std::numeric_limits<uint64_t>::max()
                ? 0
                : requested_seed + 1;
        seed_edit_->setText(QString::number(static_cast<qulonglong>(next_seed)));
    }
    refreshStatus();
}

void DebugPanel::onExportLog() {
    ExportFlightLog srv;
    srv.request.output_directory = "";
    callService(cli_export_log_, srv);
    refreshStatus();
}

void DebugPanel::onSpeedChanged(int) {
    const double s = speed_combo_->currentText().toDouble();
    callSetSpeed(s);
}

void DebugPanel::callSetSpeed(double speed) {
    SetSimSpeed srv;
    srv.request.speed = speed;
    callService(cli_speed_, srv);
    refreshStatus();
}

void DebugPanel::onToggleChanged() {
    canvas_->setShowEsdf(cb_esdf_->isChecked());
    canvas_->setShowTruthPaths(cb_truth_->isChecked());
    canvas_->setShowLocalObs(cb_local_->isChecked());
    canvas_->setShowRejected(cb_rejected_->isChecked());
}

void DebugPanel::onGoalClicked(double x, double y) {
    SetNavigationGoal srv;
    srv.request.goal[0] = x;
    srv.request.goal[1] = y;
    callService(cli_goal_, srv);
    refreshStatus();
}

void DebugPanel::onTimer() {
    ros::spinOnce();
    refreshStatus();
}

uint64_t DebugPanel::parseSeed() const {
    bool ok = false;
    const unsigned long long v = seed_edit_->text().toULongLong(&ok);
    return ok ? static_cast<uint64_t>(v) : 0;
}

std::string DebugPanel::lastServiceInfo() {
    return std::string(last_srv_ok_ ? "[OK] " : "[FAIL] ") + last_srv_result_;
}

void DebugPanel::refreshStatus() {
    if (!status_) return;
    const DebugSnapshot& s = latest_snapshot_;
    QString txt;
    txt += QString("scene_id = %1   task_id = %2   seed = %3   initialized = %4\n")
               .arg(s.scene_id)
               .arg(s.task_id)
               .arg(s.seed)
               .arg(s.simulation_initialized ? "yes" : "NO");
    txt += QString("AUTHORITATIVE PAUSE = %1\n")
               .arg(s.paused ? "PAUSED" : "RUNNING");
    txt += QString("tick = %1 (completed)   processing tick = %2   time = %3 s\n")
               .arg(s.tick)
               .arg(s.processing_tick)
               .arg(s.time, 0, 'f', 2);
    txt += QString("FSM state = %1   (prev %2)\n")
               .arg(QString::fromStdString(s.fsm_state_name))
               .arg(QString::fromStdString(s.previous_fsm_state_name));
    txt += QString("transition reason = %1   at tick = %2\n")
               .arg(QString::fromStdString(s.transition_reason))
               .arg(s.transition_tick);
    txt += QString("local target updated this tick = %1   macro tick ran = %2\n")
               .arg(s.local_target_updated_this_tick ? "yes" : "no")
               .arg(s.macro_tick_ran_this_tick ? "yes" : "no");
    txt += QString("macro active = %1   selected side = %2\n")
               .arg(s.macro_active ? "YES" : "no")
               .arg(sideName(static_cast<SideSelection>(s.side)));
    txt += QString("side reason = %1\n").arg(QString::fromStdString(s.side_reason));
    txt += QString("side avg free range (L/R) = %1 / %2 m\n")
               .arg(s.side_left_score, 0, 'f', 2)
               .arg(s.side_right_score, 0, 'f', 2);
    txt += QString("planner success = %1   failure = %2\n")
               .arg(s.planner_success ? "yes" : "NO")
               .arg(QString::fromStdString(s.planner_failure_reason));
    // ── v5: intent / output / progress / status ────────────────────
    txt += QString("planner status = %1   progress(candidate/output) = %2 / %3\n")
               .arg(QString::fromStdString(s.planner_status_name))
               .arg(s.candidate_progress_qualified ? "YES" : "no")
               .arg(s.output_progress_qualified ? "YES" : "no");
    txt += QString("intent (vx, vy, yr) = (%1, %2, %3)\n")
               .arg(s.selected_intent_vx_body, 0, 'f', 2)
               .arg(s.selected_intent_vy_body, 0, 'f', 2)
               .arg(s.selected_intent_yaw_rate, 0, 'f', 2);
    txt += QString("output (vx, vy, yr) = (%1, %2, %3)   speed = %4 m/s\n")
               .arg(s.selected_output_vx_body, 0, 'f', 2)
               .arg(s.selected_output_vy_body, 0, 'f', 2)
               .arg(s.selected_output_yaw_rate, 0, 'f', 2)
               .arg(s.selected_output_speed_mps, 0, 'f', 2);
    txt += QString("nominal progress = %1 m   executable progress = %2 m\n")
               .arg(s.nominal_progress_m, 0, 'f', 2)
               .arg(s.executable_progress_m, 0, 'f', 2);
    txt += QString("safe prefix = %1 s   stationary selected = %2 (%3)\n")
               .arg(s.safe_prefix_duration_s, 0, 'f', 2)
               .arg(s.stationary_candidate_selected ? "YES" : "no")
               .arg(QString::fromStdString(s.stationary_selection_reason));
    txt += QString("target discontinuity reset = %1   mission revision = %2\n")
               .arg(s.target_discontinuity_reset ? "YES" : "no")
               .arg(s.target_mission_revision);
    // ── v7: continuous early-avoidance risk ────────────────────────
    txt += QString("corridor blocked = %1   first blocker = %2 m   target dist = %3 m\n")
               .arg(s.local_corridor_blocked ? "YES" : "no")
               .arg(s.first_blocking_obstacle_distance, 0, 'f', 2)
               .arg(s.local_target_distance, 0, 'f', 2);
    txt += QString("risk = %1   avoidance = %2   active = %3\n")
               .arg(s.obstacle_risk_cost, 0, 'f', 3)
               .arg(s.avoidance_strength, 0, 'f', 3)
               .arg(s.avoidance_active ? "YES" : "no");
    txt += QString("predicted closest clearance = %1 m   TTC = %2 s\n")
               .arg(s.predicted_closest_clearance, 0, 'f', 2)
               .arg(s.time_to_collision, 0, 'f', 2);
    // ── v7: rolling macro guide ────────────────────────────────────
    txt += QString("macro route progress = %1 m   guide lookahead = %2 m\n")
               .arg(s.macro_route_progress, 0, 'f', 2)
               .arg(s.macro_guide_lookahead, 0, 'f', 2);
    txt += QString("guide update = %1   macro no-progress = %2 s\n")
               .arg(QString::fromStdString(s.macro_guide_update_reason))
               .arg(s.macro_no_progress_duration, 0, 'f', 2);
    txt += QString("consecutive 30Hz failures = %1\n").arg(s.consecutive_failures_30hz);
    txt += QString("rejected = %1  (not_known_free=%2, outside_fov=%3, "
                   "clearance=%4, no_progress=%5, other=%6, brake_clear=%7)\n")
               .arg(s.rejected_candidate_count)
               .arg(s.reject_not_known_free)
               .arg(s.reject_outside_current_fov)
               .arg(s.reject_observed_clearance_too_small)
               .arg(s.reject_no_progress)
               .arg(s.reject_other)
               .arg(s.reject_insufficient_braking_clearance);
    txt += QString("UNKNOWN recovery = %1 ticks   active = %2   episodes = %3\n")
               .arg(s.unknown_recovery_ticks)
               .arg(s.unknown_recovery_active ? "YES" : "no")
               .arg(s.unknown_recovery_episode_count);
    txt += QString("target bearing err = %1 deg   terminal heading err = %2 deg\n")
               .arg(s.target_bearing_error_deg, 0, 'f', 1)
               .arg(s.selected_terminal_heading_error_deg, 0, 'f', 1);
    txt += QString("vel alignment err = %1 deg   cross-track = %2 m\n")
               .arg(s.selected_velocity_alignment_error_deg, 0, 'f', 1)
               .arg(s.selected_cross_track_error_m, 0, 'f', 2);
    txt += QString("cost = %1 (prog %2, clear %3, smooth %4, speed %5, "
                   "yawRate %6)\n")
               .arg(s.selected_cost_total, 0, 'f', 3)
               .arg(s.selected_cost_progress, 0, 'f', 3)
               .arg(s.selected_cost_clearance, 0, 'f', 3)
               .arg(s.selected_cost_smoothness, 0, 'f', 3)
               .arg(s.selected_cost_speed_change, 0, 'f', 3)
               .arg(s.selected_cost_yaw_rate_change, 0, 'f', 3);
    txt += QString("cost(head %1, velAl %2, cross %3)\n")
               .arg(s.selected_cost_terminal_heading, 0, 'f', 3)
               .arg(s.selected_cost_velocity_alignment, 0, 'f', 3)
               .arg(s.selected_cost_cross_track, 0, 'f', 3);
    txt += QString("local target update event = %1   macro tick event = %2\n")
               .arg(s.local_target_update_event)
               .arg(s.macro_tick_event);
    txt += QString("accepted goal event = %1   reentry guard left = %2 ticks\n")
               .arg(s.accepted_goal_event)
               .arg(s.macro_reentry_guard_ticks);
    txt += QString("current clearance (observed/truth) = %1 / %2 m\n")
               .arg(s.current_observed_clearance, 0, 'f', 3)
               .arg(s.current_truth_clearance, 0, 'f', 3);
    txt += QString("episode min clearance (observed/truth) = %1 / %2 m\n")
               .arg(s.min_observed_clearance, 0, 'f', 3)
               .arg(s.truth_min_clearance, 0, 'f', 3);
    // ── Soft-clearance / dynamic-envelope diagnostics (v4) ─────────
    // NaN means "no candidate selected this tick" — shown as n/a so a
    // missing measurement can never be read as a real 0.
    auto fmtOrNa = [](double v) {
        return std::isnan(v) ? QString("n/a") : QString::number(v, 'f', 3);
    };
    txt += QString("soft min clearance = %1 m   handoff clearance = %2 m\n")
               .arg(fmtOrNa(s.selected_soft_min_clearance_m))
               .arg(fmtOrNa(s.handoff_clearance_m));
    txt += QString("dynamic required clearance = %1 m   closing speed = %2 m/s\n")
               .arg(fmtOrNa(s.selected_dynamic_required_clearance_m))
               .arg(fmtOrNa(s.selected_closing_speed_mps));
    {
        // GUI shows the actual measured clearance, the dynamically
        // required clearance and their difference (negative → the dynamic
        // envelope is violated).
        const double soft = s.selected_soft_min_clearance_m;
        const double req = s.selected_dynamic_required_clearance_m;
        if (!std::isnan(soft) && !std::isnan(req)) {
            txt += QString("clearance margin (actual - required) = %1 m\n")
                       .arg(soft - req, 0, 'f', 3);
        }
    }
    txt += QString("dynamic clearance blocked = %1   limit cycle detected = %2\n")
               .arg(s.dynamic_clearance_blocked ? "YES" : "no")
               .arg(s.local_limit_cycle_detected ? "YES" : "no");
    txt += QString("dynamic window candidates = %1   start clearance recovery = %2\n")
               .arg(s.dynamic_window_candidate_count)
               .arg(s.start_clearance_recovery_used ? "YES" : "no");
    txt += QString("blocker id = %1   association = %2\n")
               .arg(s.blocker_id)
               .arg(QString::fromStdString(blockerAssociationName(
                   static_cast<BlockerAssociation>(s.blocker_association))));
    txt += QString("blocker_passed_latched = %1\n")
               .arg(s.blocker_passed_latched ? "YES" : "no");
    txt += QString("fixed-route progress = %1 m   blocker progress = %2 m\n")
               .arg(s.entry_vehicle_progress, 0, 'f', 2)
               .arg(s.entry_blocker_progress, 0, 'f', 2);
    txt += QString("projection dist = %1 m   segment = %2\n")
               .arg(s.entry_projection_dist, 0, 'f', 2)
               .arg(s.entry_segment_index);
    txt += QString("progress delta = %1 m   max/tick = %2 m\n")
               .arg(s.entry_progress_delta, 0, 'f', 2)
               .arg(s.entry_progress_max_delta, 0, 'f', 2);
    txt += QString("macro stable-exit count = %1\n")
               .arg(s.macro_stable_exit_count);
    txt += QString("pending goal set = %1   revision = %2   pos = (%3, %4)\n")
               .arg(s.pending_goal_set ? "YES" : "no")
               .arg(s.pending_goal_revision)
               .arg(s.pending_goal_position[0], 0, 'f', 2)
               .arg(s.pending_goal_position[1], 0, 'f', 2);
    txt += QString("goal_reached = %1  collision = %2  task_invalid = %3\n")
               .arg(s.goal_reached ? "YES" : "no")
               .arg(s.collision ? "YES" : "no")
               .arg(s.task_invalid ? "YES" : "no");

    // ── Audit fields (information-leak / timing inspection) ─────────
    txt += "\n── AUDIT ──\n";
    txt += QString("used_truth_by_local_planner = %1\n")
               .arg(s.used_truth_by_local_planner ? "TRUE (LEAK!)" : "false");
    txt += QString("used_global_esdf_by_local_planner = %1\n")
               .arg(s.used_global_esdf_by_local_planner ? "TRUE (LEAK!)" : "false");
    txt += QString("used_global_path_by_local_planner = %1\n")
               .arg(s.used_global_path_by_local_planner ? "TRUE (LEAK!)" : "false");
    txt += QString("macro_used_privileged_esdf = %1\n")
               .arg(s.macro_used_privileged_esdf ? "true" : "false");
    txt += QString("side_selected_from_visible_evidence = %1\n")
               .arg(s.side_selected_from_visible_evidence ? "true" : "false");
    txt += QString("side_selected_using_CURRENT_PATCH = %1\n")
               .arg(s.side_selected_using_current_patch ? "true" : "false");
    txt += QString("side_ambiguous_defaulted_right = %1\n")
               .arg(s.side_ambiguous_defaulted_right ? "true" : "false");
    txt += QString("macro_enter_event = %1   macro_exit_event = %2\n")
               .arg(s.macro_enter_event)
               .arg(s.macro_exit_event);
    txt += QString("obstacle_first_observed_event = %1\n")
               .arg(s.obstacle_first_observed_event);
    txt += QString("immediate_avoidance_event = %1   emergency_brake_event = %2\n")
               .arg(s.immediate_avoidance_event)
               .arg(s.emergency_brake_event);

    txt += QString("\nlast service: %1\n")
               .arg(QString::fromStdString(lastServiceInfo()));
    status_->setPlainText(txt);
}

}  // namespace il_2d_multiscale_debug
