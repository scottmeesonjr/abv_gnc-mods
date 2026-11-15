#pragma once

#include <QElapsedTimer>
#include <QLabel>
#include <QTimer>
#include <QVariant>
#include <QWidget>

// Docking mechanism status (abv/dock/status) and the SYS.01 velocity alert
// (abv/nav/alert). Fed through TopicAdapter signals so every widget update
// runs on the GUI thread.
class DockPanel : public QWidget
{
    Q_OBJECT
public:
    explicit DockPanel(QWidget* parent = nullptr);

public slots:
    // QVector<double>{state, locked, link_ok, estop, cycle_count, fault_code}
    void onDockStatus(const QVariant& aData);
    // QVector<double>{speed, closing_velocity, range, alert}
    void onNavAlert(const QVariant& aData);
    // QString from abv/dock/event, e.g. "CYCLE|2|CAPTURED|0.0510",
    // "COMPLETE|10|10" or "ABORTED|<reason>". COMPLETE / ABORTED pop up a
    // (non-blocking) message box; CYCLE updates the "Last cycle" row.
    void onDockEvent(const QVariant& aData);

private:
    void checkStale();

    QLabel* mMechanism{nullptr};
    QLabel* mCycles{nullptr};
    QLabel* mApproach{nullptr};
    QLabel* mLastCycle{nullptr};
    QLabel* mAlert{nullptr};

    QElapsedTimer mLastStatus;
    QTimer mStaleTimer;
};
