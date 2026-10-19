#pragma once

#include <QWidget>
#include <QVariant>
#include <QString>
#include <QPointF>
#include <QRectF>
#include <QVector>

#include "abv_common/Configurations.h"

// Plain world-frame (x, y, radius) obstacle, used purely for rendering the
// persistent obstacle set - main.cpp owns the authoritative list (since it's
// the one publishing to abv/scene/obstacles) and pushes it here via
// setObstacles after every add/clear.
struct PlacedObstacle
{
    double mX;
    double mY;
    double mRadius;
};

// Top-down, to-scale render of the physical table with the robot's live
// pose: gridded table surface and a heading-aware robot glyph (square
// aluminum-extrusion body + front camera housing + thruster nozzles that
// light up when firing). Pose arrives via onPoseUpdate, meant to be
// connected to a TopicAdapter<AbvState, QVector<double>>'s newDataVariant
// signal (see navigationStateConvertor) so updates land on the GUI thread
// rather than the ROS subscription thread. Thruster state arrives via
// onThrusterState the same way, from AbvThrusterStatus.thrusters.
class TableTopView : public QWidget
{
    Q_OBJECT
public:
    explicit TableTopView(const TableViewConfig& aConfig, QWidget* parent = nullptr);

    // Replaces the persistent (already-committed) obstacle set drawn on the
    // table. This widget doesn't track obstacle state itself - the caller is
    // the source of truth, same split as goalPoseSelected/clearGoalGhost.
    void setObstacles(const QVector<PlacedObstacle>& aObstacles);

public slots:
    // Expects a QVector<double>{x, y, yaw, valid} as produced by
    // conversions::navigationStateConvertor.
    void onPoseUpdate(const QVariant& aData);

    // Expects a QString holding the 8-char '0'/'1' AbvThrusterStatus.thrusters
    // string, index i = thruster (i+1) (see Control.Thrusters.Allocation).
    void onThrusterState(const QVariant& aData);

    // Expects a QVector<QPointF> of world-frame (x, y) points, as produced by
    // conversions::pathConvertor - the remaining path abv_guidance is
    // currently executing. Empty clears the drawn path.
    void onPathUpdate(const QVariant& aData);

    // Hides the proposed-goal ghost. Called after the caller has resolved
    // (sent or cancelled) a goalPoseSelected signal.
    void clearGoalGhost();

    // Hides the proposed-obstacle ghost. Called after the caller has resolved
    // (added or cancelled) an obstaclePlaced signal.
    void clearObstacleGhost();

signals:
    // Emitted on mouse release after a left-click-drag gesture on the table.
    // This widget doesn't know about ROS/CommandPanel - the caller decides
    // what "sending" a goal pose means (see main.cpp).
    void goalPoseSelected(double aX, double aY, double aYaw);

    // Emitted on mouse release after a right-click-drag gesture on the table
    // (i.e. one that actually moved - see mouseReleaseEvent/mRightDragExceededThreshold).
    // As with goalPoseSelected, this widget doesn't know about ROS - the
    // caller decides whether/how to publish it (see main.cpp).
    void obstaclePlaced(double aX, double aY, double aRadius);

    // Emitted when the user chooses "Clear Obstacles" from the menu shown on
    // a plain right-click (no drag). This widget doesn't own the obstacle
    // list - the caller (main.cpp) is the source of truth and decides how to
    // react.
    void clearObstaclesRequested();

protected:
    void paintEvent(QPaintEvent* aEvent) override;
    void mousePressEvent(QMouseEvent* aEvent) override;
    void mouseMoveEvent(QMouseEvent* aEvent) override;
    void mouseReleaseEvent(QMouseEvent* aEvent) override;

private:
    QRectF tableToWidget() const;
    double worldScale(const QRectF& aTableRect) const;
    QPointF worldToPixel(const QRectF& aTableRect, double aX, double aY) const;
    QPointF pixelToWorld(const QRectF& aTableRect, const QPointF& aPixel) const;

    void drawGrid(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawPath(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawRobot(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawObstacles(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawGoalGhost(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawObstacleGhost(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawReadout(QPainter& aPainter) const;
    void drawLegend(QPainter& aPainter) const;

    TableViewConfig mConfig;

    double mX{0.0};
    double mY{0.0};
    double mYaw{0.0};
    bool mHasPose{false};

    QString mThrusterState{"00000000"};

    bool mDraggingGoal{false};
    bool mHasGoalGhost{false};
    double mGoalX{0.0};
    double mGoalY{0.0};
    double mGoalYaw{0.0};
    QPointF mDragStartWorld;

    bool mDraggingObstacle{false};
    bool mHasObstacleGhost{false};
    bool mRightDragExceededThreshold{false};
    double mObstacleX{0.0};
    double mObstacleY{0.0};
    double mObstacleRadius{0.0};

    QVector<PlacedObstacle> mObstacles;

    // World-frame (x, y) points of the remaining path abv_guidance is
    // currently executing, from onPathUpdate.
    QVector<QPointF> mPath;
};
