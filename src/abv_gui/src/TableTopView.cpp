#include "abv_gui/TableTopView.h"

#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QMouseEvent>
#include <QMenu>
#include <QCursor>
#include <cmath>

namespace
{
    constexpr double kMargin = 28.0;       // px, room for the border/labels
    constexpr double kGridSpacing = 0.3;   // meters
    constexpr double kEps = 1e-6;
    constexpr double kMinDragPixels = 18.0; // below this, treat a drag as "no heading intent" -> yaw 0
}

TableTopView::TableTopView(const TableViewConfig& aConfig, QWidget* parent)
    : QWidget(parent), mConfig(aConfig)
{
    setMinimumSize(300, 200);

    // Right-click can resolve to either an obstacle-placement drag or a
    // plain click-to-clear depending on what happens before release (see
    // mouseReleaseEvent) - handled entirely by us, so disable Qt's own
    // native context-menu delivery to avoid it firing alongside/instead of
    // our manual handling.
    setContextMenuPolicy(Qt::NoContextMenu);
}

void TableTopView::onPoseUpdate(const QVariant& aData)
{
    QVector<double> pose = aData.value<QVector<double>>();
    if (pose.size() != 4)
    {
        return;
    }

    mX = pose[0];
    mY = pose[1];
    mYaw = pose[2];
    mHasPose = true;

    update();
}

void TableTopView::onThrusterState(const QVariant& aData)
{
    QString state = aData.value<QString>();
    if (state.size() != 8)
    {
        return;
    }

    mThrusterState = state;
    update();
}

void TableTopView::onPathUpdate(const QVariant& aData)
{
    mPath = aData.value<QVector<QPointF>>();
    update();
}

void TableTopView::clearGoalGhost()
{
    mHasGoalGhost = false;
    update();
}

void TableTopView::clearObstacleGhost()
{
    mHasObstacleGhost = false;
    update();
}

void TableTopView::setObstacles(const QVector<PlacedObstacle>& aObstacles)
{
    mObstacles = aObstacles;
    update();
}

QRectF TableTopView::tableToWidget() const
{
    QRectF avail = rect().adjusted(kMargin, kMargin, -kMargin, -kMargin);
    if (avail.width() <= 0 || avail.height() <= 0 || mConfig.mWidth <= 0 || mConfig.mHeight <= 0)
    {
        return QRectF();
    }

    // mWidth (x, the long 3.6m edge) renders vertically; mHeight (y, the
    // short 1.8m edge) renders horizontally - see worldToPixel.
    double scale = std::min(avail.width() / mConfig.mHeight, avail.height() / mConfig.mWidth);
    double w = mConfig.mHeight * scale;
    double h = mConfig.mWidth * scale;

    QPointF topLeft(avail.center().x() - w / 2.0, avail.center().y() - h / 2.0);
    return QRectF(topLeft, QSizeF(w, h));
}

// Pixels per meter for the current layout - uniform in both directions, so
// this also doubles as the conversion factor for physical lengths (glyph
// dimensions, lens radius, ...) that never go through worldToPixel as a point.
double TableTopView::worldScale(const QRectF& aTableRect) const
{
    return aTableRect.height() / mConfig.mWidth;
}

// World frame: origin at the table's bottom-right corner (the physical rig's
// calibrated zero), x and y both increasing as the robot moves onto the
// table - x along the long (3.6m) edge, y along the short (1.8m) edge - the
// same convention AbvState/AbvVec3 use. On screen the long edge runs
// vertically (x increasing = up) and the short edge runs horizontally (y
// increasing = left, off the right edge where the origin sits). This is the
// one place that mapping happens; every drawing routine below goes through
// it so there's exactly one spot to get the sign/axis-swap right.
QPointF TableTopView::worldToPixel(const QRectF& aTableRect, double aX, double aY) const
{
    double scale = worldScale(aTableRect);
    return QPointF(
        aTableRect.right() - aY * scale,
        aTableRect.bottom() - aX * scale);
}

// Algebraic inverse of worldToPixel - same single spot for the axis mapping,
// just solved the other way.
QPointF TableTopView::pixelToWorld(const QRectF& aTableRect, const QPointF& aPixel) const
{
    double scale = worldScale(aTableRect);
    return QPointF(
        (aTableRect.bottom() - aPixel.y()) / scale,
        (aTableRect.right() - aPixel.x()) / scale);
}

void TableTopView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(20, 20, 20));

    QRectF tableRect = tableToWidget();
    if (!tableRect.isValid())
    {
        return;
    }

    drawGrid(p, tableRect);

    p.setPen(QPen(QColor(100, 100, 100), 2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(tableRect);

    drawPath(p, tableRect);

    if (mHasPose)
    {
        drawRobot(p, tableRect);
    }

    drawObstacles(p, tableRect);

    if (mHasGoalGhost)
    {
        drawGoalGhost(p, tableRect);
    }

    if (mHasObstacleGhost)
    {
        drawObstacleGhost(p, tableRect);
    }

    drawReadout(p);
    drawLegend(p);
}

void TableTopView::mousePressEvent(QMouseEvent* aEvent)
{
    QRectF tableRect = tableToWidget();
    if (!tableRect.isValid() || !tableRect.contains(aEvent->pos()))
    {
        QWidget::mousePressEvent(aEvent);
        return;
    }

    QPointF world = pixelToWorld(tableRect, aEvent->pos());

    if (aEvent->button() == Qt::LeftButton)
    {
        mDragStartWorld = world;
        mGoalX = world.x();
        mGoalY = world.y();
        mGoalYaw = 0.0;
        mDraggingGoal = true;
        mHasGoalGhost = true;
    }
    else if (aEvent->button() == Qt::RightButton)
    {
        // Don't show the obstacle ghost yet - a plain click (no drag) means
        // "clear obstacles" instead, decided on release (see mouseReleaseEvent).
        mObstacleX = world.x();
        mObstacleY = world.y();
        mObstacleRadius = 0.0;
        mDraggingObstacle = true;
        mRightDragExceededThreshold = false;
    }
    else
    {
        QWidget::mousePressEvent(aEvent);
        return;
    }

    update();
}

void TableTopView::mouseMoveEvent(QMouseEvent* aEvent)
{
    if (!mDraggingGoal && !mDraggingObstacle)
    {
        QWidget::mouseMoveEvent(aEvent);
        return;
    }

    QRectF tableRect = tableToWidget();
    QPointF world = pixelToWorld(tableRect, aEvent->pos());

    if (mDraggingGoal)
    {
        double dx = world.x() - mDragStartWorld.x();
        double dy = world.y() - mDragStartWorld.y();
        if (std::hypot(dx, dy) * worldScale(tableRect) >= kMinDragPixels)
        {
            mGoalYaw = std::atan2(dy, dx);
        }
    }
    else if (mDraggingObstacle)
    {
        double radius = std::hypot(world.x() - mObstacleX, world.y() - mObstacleY);
        if (radius * worldScale(tableRect) >= kMinDragPixels)
        {
            mRightDragExceededThreshold = true;
            mHasObstacleGhost = true;
        }
        mObstacleRadius = radius;
    }

    update();
}

void TableTopView::mouseReleaseEvent(QMouseEvent* aEvent)
{
    if (!mDraggingGoal && !mDraggingObstacle)
    {
        QWidget::mouseReleaseEvent(aEvent);
        return;
    }

    if (mDraggingGoal)
    {
        mDraggingGoal = false;
        emit goalPoseSelected(mGoalX, mGoalY, mGoalYaw);
    }
    else if (mDraggingObstacle)
    {
        mDraggingObstacle = false;

        if (mRightDragExceededThreshold)
        {
            emit obstaclePlaced(mObstacleX, mObstacleY, mObstacleRadius);
        }
        else
        {
            // Plain right-click, no drag - offer to clear instead.
            QMenu menu;
            QAction* clearAction = menu.addAction("Clear Obstacles");

            QAction* chosen = menu.exec(QCursor::pos());
            if (chosen == clearAction)
            {
                emit clearObstaclesRequested();
            }
        }
    }
}

void TableTopView::drawGrid(QPainter& aPainter, const QRectF& aTableRect) const
{
    aPainter.save();
    aPainter.setClipRect(aTableRect);

    QPen minorPen(QColor(50, 50, 50));
    minorPen.setWidthF(1.0);
    aPainter.setPen(minorPen);

    // lines of constant x (perpendicular to the long axis) - horizontal on screen
    for (double gx = 0.0; gx <= mConfig.mWidth + kEps; gx += kGridSpacing)
    {
        QPointF left = worldToPixel(aTableRect, gx, mConfig.mHeight);
        QPointF right = worldToPixel(aTableRect, gx, 0.0);
        aPainter.drawLine(left, right);
    }

    // lines of constant y (perpendicular to the short axis) - vertical on screen
    for (double gy = 0.0; gy <= mConfig.mHeight + kEps; gy += kGridSpacing)
    {
        QPointF top = worldToPixel(aTableRect, mConfig.mWidth, gy);
        QPointF bottom = worldToPixel(aTableRect, 0.0, gy);
        aPainter.drawLine(top, bottom);
    }

    // origin marker at the bottom-right corner
    aPainter.setPen(Qt::NoPen);
    aPainter.setBrush(QColor(140, 140, 140));
    QPointF origin = worldToPixel(aTableRect, 0.0, 0.0);
    aPainter.drawEllipse(origin, 3.5, 3.5);

    aPainter.restore();
}

void TableTopView::drawPath(QPainter& aPainter, const QRectF& aTableRect) const
{
    if (!mHasPose || mPath.isEmpty())
    {
        return;
    }

    // Anchor the line to the robot's own live position rather than whatever
    // (possibly stale) first point abv_guidance published - see
    // StraightLineGenerator::getPath().
    QPolygonF pixels;
    pixels.reserve(mPath.size() + 1);
    pixels << worldToPixel(aTableRect, mX, mY);
    for (const QPointF& worldPt : mPath)
    {
        pixels << worldToPixel(aTableRect, worldPt.x(), worldPt.y());
    }

    aPainter.save();

    // Tesla-style planned-path line: a soft, wider translucent glow pass
    // underneath a crisp solid line on top, drawn before the robot glyph so
    // the vehicle renders above the path it's following.
    QPen glowPen(QColor(60, 140, 255, 60), 13.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    aPainter.setPen(glowPen);
    aPainter.setBrush(Qt::NoBrush);
    aPainter.drawPolyline(pixels);

    QPen linePen(QColor(70, 150, 255), 4.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    aPainter.setPen(linePen);
    aPainter.drawPolyline(pixels);

    aPainter.restore();
}

void TableTopView::drawRobot(QPainter& aPainter, const QRectF& aTableRect) const
{
    double halfL = mConfig.mRobotLength / 2.0;
    double halfW = mConfig.mRobotWidth / 2.0;

    // Square aluminum-extrusion body, +x forward, defined in the robot's
    // local (body) frame, meters. Rotated by yaw and translated to the
    // current world pose, then each vertex goes through worldToPixel
    // individually so the body can't end up with a different handedness
    // than the grid.
    QVector<QPointF> bodyLocal = {
        {halfL,  halfW},
        {halfL, -halfW},
        {-halfL, -halfW},
        {-halfL,  halfW},
    };

    // Small corner-bracket accents, inset from each corner, to suggest
    // bolted extrusion corner plates.
    double accent = std::min(halfL, halfW) * 0.28;
    QVector<QPointF> corners = {
        {halfL, halfW}, {halfL, -halfW}, {-halfL, -halfW}, {-halfL, halfW}
    };

    double cosY = std::cos(mYaw);
    double sinY = std::sin(mYaw);

    auto toPixel = [&](const QPointF& local) {
        double wx = mX + local.x() * cosY - local.y() * sinY;
        double wy = mY + local.x() * sinY + local.y() * cosY;
        return worldToPixel(aTableRect, wx, wy);
    };

    // Pointy thruster nozzles, one pair per corner (not per side): each
    // corner has one nozzle pointing along +/-x and one along +/-y, matching
    // the real hardware layout (thruster diagram: T4/T5 on the front-left
    // corner, T3/T2 on front-right, T1/T8 on back-right, T6/T7 on
    // back-left). Ordered by index 1-8 so mThrusterState[i] lines up with
    // thrusterLocal[i] directly.
    double thrusterRadius = std::min(halfL, halfW) * 0.12;
    double thrusterPoke = thrusterRadius * 0.9;
    double frontBackOffset = halfW * 0.6;
    double leftRightOffset = halfL * 0.6;
    struct Thruster { QPointF base; QPointF outward; };
    QVector<Thruster> thrusterLocal = {
        {{-leftRightOffset, -halfW}, {0, -1}}, // T1 - right edge, back
        {{ leftRightOffset, -halfW}, {0, -1}}, // T2 - right edge, front
        {{ halfL, -frontBackOffset}, {1, 0}},  // T3 - front edge, right
        {{ halfL,  frontBackOffset}, {1, 0}},  // T4 - front edge, left
        {{ leftRightOffset,  halfW}, {0, 1}},  // T5 - left edge, front
        {{-leftRightOffset,  halfW}, {0, 1}},  // T6 - left edge, back
        {{-halfL,  frontBackOffset}, {-1, 0}}, // T7 - back edge, left
        {{-halfL, -frontBackOffset}, {-1, 0}}, // T8 - back edge, right
    };

    // Small housing bump, inset into the front edge rather than sitting
    // fully proud of it, giving a recessed-mount look instead of a bump
    // sticking squarely out.
    double housingDepth = std::min(halfL, halfW) * 0.32;
    double housingHalfW = halfW * 0.22;
    double housingInset = housingDepth * 0.45;
    QVector<QPointF> housingLocal = {
        {halfL - housingInset, housingHalfW},
        {halfL - housingInset + housingDepth, housingHalfW},
        {halfL - housingInset + housingDepth, -housingHalfW},
        {halfL - housingInset, -housingHalfW},
    };

    // Field-of-view wedge fanning out from the tip of the housing - the
    // standard way to show a camera's facing direction from directly above.
    QPointF wedgeOrigin(halfL - housingInset + housingDepth, 0.0);
    double wedgeLength = mConfig.mRobotLength * 0.9;
    double wedgeHalfAngle = 32.0 * M_PI / 180.0;
    double wedgeReach = wedgeLength * std::sin(wedgeHalfAngle);
    QVector<QPointF> wedgeLocal = {
        wedgeOrigin,
        {wedgeOrigin.x() + wedgeLength * std::cos(wedgeHalfAngle),  wedgeReach},
        {wedgeOrigin.x() + wedgeLength * std::cos(wedgeHalfAngle), -wedgeReach},
    };

    QPolygonF bodyPixels;
    for (const auto& v : bodyLocal) bodyPixels << toPixel(v);

    QPolygonF housingPixels;
    for (const auto& v : housingLocal) housingPixels << toPixel(v);

    QPolygonF wedgePixels;
    for (const auto& v : wedgeLocal) wedgePixels << toPixel(v);

    aPainter.save();

    // soft drop shadow for a bit of depth
    QPointF centerPixel = toPixel(QPointF(0.0, 0.0));
    aPainter.setPen(Qt::NoPen);
    aPainter.setBrush(QColor(0, 0, 0, 90));
    double shadowR = std::max(bodyPixels.boundingRect().width(), bodyPixels.boundingRect().height()) * 0.6;
    aPainter.drawEllipse(centerPixel + QPointF(2, 3), shadowR, shadowR * 0.75);

    // FOV wedge, drawn first so the body/housing sit on top of its base
    aPainter.setPen(Qt::NoPen);
    aPainter.setBrush(QColor(210, 230, 255, 30));
    aPainter.drawPolygon(wedgePixels);

    // body - bare aluminum extrusion
    aPainter.setPen(QPen(QColor(80, 80, 85), 1.5));
    aPainter.setBrush(QColor(150, 150, 155));
    aPainter.drawPolygon(bodyPixels);

    // pointy thruster nozzles - lit up orange with a soft glow when firing
    double scale = worldScale(aTableRect);
    bool haveThrusterState = mThrusterState.size() == 8;
    for (int i = 0; i < thrusterLocal.size(); ++i)
    {
        const Thruster& t = thrusterLocal[i];
        bool firing = haveThrusterState && mThrusterState[i] == QChar('1');

        QPointF tangent(-t.outward.y(), t.outward.x());
        QPointF baseCenter = t.base - t.outward * (thrusterRadius * 0.3);
        QPointF apex = t.base + t.outward * (thrusterPoke + thrusterRadius * 1.4);

        if (firing)
        {
            aPainter.setPen(Qt::NoPen);
            aPainter.setBrush(QColor(255, 140, 40, 70));
            aPainter.drawEllipse(toPixel(t.base), thrusterRadius * 2.2 * scale, thrusterRadius * 2.2 * scale);
        }

        aPainter.setPen(QPen(firing ? QColor(255, 190, 100) : QColor(70, 80, 95), 1.0));
        aPainter.setBrush(firing ? QColor(255, 140, 40) : QColor(25, 30, 40));

        QPolygonF nozzle;
        nozzle << toPixel(apex)
               << toPixel(baseCenter + tangent * (thrusterRadius * 0.45))
               << toPixel(baseCenter - tangent * (thrusterRadius * 0.45));
        aPainter.drawPolygon(nozzle);
    }

    // corner brackets
    aPainter.setPen(Qt::NoPen);
    aPainter.setBrush(QColor(95, 95, 100));
    for (const auto& corner : corners)
    {
        QPointF inX(corner.x() - (corner.x() > 0 ? accent : -accent), corner.y());
        QPointF inY(corner.x(), corner.y() - (corner.y() > 0 ? accent : -accent));

        QPolygonF bracket;
        bracket << toPixel(corner) << toPixel(inX) << toPixel(QPointF(inX.x(), inY.y())) << toPixel(inY);
        aPainter.drawPolygon(bracket);
    }

    // camera housing, protruding past the front edge - no lens drawn on it
    // (a shaded circle here reads as a 3D dome sticking up rather than a
    // flat top-down camera, so the housing + FOV wedge carry "front" alone)
    aPainter.setPen(QPen(QColor(60, 60, 65), 1.0));
    aPainter.setBrush(QColor(110, 110, 115));
    aPainter.drawPolygon(housingPixels);

    aPainter.restore();
}

void TableTopView::drawObstacles(QPainter& aPainter, const QRectF& aTableRect) const
{
    if (mObstacles.isEmpty())
    {
        return;
    }

    double scale = worldScale(aTableRect);

    aPainter.save();
    aPainter.setPen(QPen(QColor(220, 80, 60), 1.5));
    aPainter.setBrush(QColor(220, 80, 60, 90));

    for (const PlacedObstacle& obstacle : mObstacles)
    {
        QPointF center = worldToPixel(aTableRect, obstacle.mX, obstacle.mY);
        double r = obstacle.mRadius * scale;
        aPainter.drawEllipse(center, r, r);
    }

    aPainter.restore();
}

void TableTopView::drawGoalGhost(QPainter& aPainter, const QRectF& aTableRect) const
{
    double halfL = mConfig.mRobotLength / 2.0;
    double halfW = mConfig.mRobotWidth / 2.0;

    double cosY = std::cos(mGoalYaw);
    double sinY = std::sin(mGoalYaw);

    auto toPixel = [&](const QPointF& local) {
        double wx = mGoalX + local.x() * cosY - local.y() * sinY;
        double wy = mGoalY + local.x() * sinY + local.y() * cosY;
        return worldToPixel(aTableRect, wx, wy);
    };

    // Deliberately simpler than the live robot glyph - a dashed outline
    // square plus a heading line - so a proposed goal reads as a translucent
    // preview rather than competing visually with the real vehicle.
    QVector<QPointF> bodyLocal = {
        {halfL,  halfW}, {halfL, -halfW}, {-halfL, -halfW}, {-halfL,  halfW}
    };

    QPolygonF bodyPixels;
    for (const auto& v : bodyLocal) bodyPixels << toPixel(v);

    aPainter.save();

    QColor ghostColor(120, 200, 255);

    QPen dashedPen(ghostColor, 1.5, Qt::DashLine);
    aPainter.setPen(dashedPen);
    aPainter.setBrush(QColor(120, 200, 255, 40));
    aPainter.drawPolygon(bodyPixels);

    QPen headingPen(ghostColor, 2.0);
    aPainter.setPen(headingPen);
    aPainter.drawLine(toPixel(QPointF(0.0, 0.0)), toPixel(QPointF(halfL * 1.4, 0.0)));

    aPainter.restore();
}

void TableTopView::drawObstacleGhost(QPainter& aPainter, const QRectF& aTableRect) const
{
    double scale = worldScale(aTableRect);
    QPointF center = worldToPixel(aTableRect, mObstacleX, mObstacleY);
    double r = mObstacleRadius * scale;

    aPainter.save();

    QColor ghostColor(255, 140, 100);
    aPainter.setPen(QPen(ghostColor, 1.5, Qt::DashLine));
    aPainter.setBrush(QColor(255, 140, 100, 50));
    aPainter.drawEllipse(center, r, r);

    aPainter.restore();
}

void TableTopView::drawReadout(QPainter& aPainter) const
{
    QString text = mHasPose
        ? QString("X: %1 m    Y: %2 m    Yaw: %3 deg")
              .arg(mX, 0, 'f', 3)
              .arg(mY, 0, 'f', 3)
              .arg(mYaw * 180.0 / M_PI, 0, 'f', 1)
        : QString("Waiting for pose...");

    QFont font = aPainter.font();
    font.setPointSize(9);
    aPainter.setFont(font);

    QFontMetrics fm(font);
    QRectF box(8, 8, fm.horizontalAdvance(text) + 16, fm.height() + 8);

    aPainter.save();
    aPainter.setPen(Qt::NoPen);
    aPainter.setBrush(QColor(30, 30, 30, 200));
    aPainter.drawRoundedRect(box, 4, 4);

    aPainter.setPen(Qt::white);
    aPainter.drawText(box, Qt::AlignCenter, text);
    aPainter.restore();
}

void TableTopView::drawLegend(QPainter& aPainter) const
{
    // Stacked short lines rather than one long line - a single line this
    // long ran off the right edge on narrower windows even at a small point
    // size, since its width doesn't depend on the widget's actual width.
    static const QVector<QString> lines = {
        "Left-drag: Set Goal",
        "Right-drag: Place Obstacle",
        "Right-click: Clear Obstacles"
    };

    QFont font = aPainter.font();
    font.setPointSize(8);
    aPainter.setFont(font);

    QFontMetrics fm(font);
    double textWidth = 0.0;
    for (const QString& line : lines)
    {
        textWidth = std::max(textWidth, static_cast<double>(fm.horizontalAdvance(line)));
    }

    // Clamp to the widget's own width as a last resort, so the box itself
    // never extends past the visible area even on a very narrow window.
    double boxWidth = std::min(textWidth + 16.0, static_cast<double>(width()) - 16.0);
    double lineHeight = fm.height();
    double boxHeight = lineHeight * lines.size() + 10.0;

    // Bottom-left of the widget rect (not the table rect), so it stays put
    // and visible regardless of table scaling/margins - same corner-overlay
    // approach as drawReadout, just anchored to the opposite corner so the
    // two never overlap.
    QRectF box(8, height() - boxHeight - 8, boxWidth, boxHeight);

    aPainter.save();
    aPainter.setPen(Qt::NoPen);
    aPainter.setBrush(QColor(30, 30, 30, 200));
    aPainter.drawRoundedRect(box, 4, 4);

    aPainter.setPen(QColor(180, 180, 180));
    for (int i = 0; i < lines.size(); ++i)
    {
        QRectF lineRect(box.left() + 8, box.top() + 5 + i * lineHeight, box.width() - 16, lineHeight);
        aPainter.drawText(lineRect, Qt::AlignLeft | Qt::AlignVCenter, lines[i]);
    }
    aPainter.restore();
}
