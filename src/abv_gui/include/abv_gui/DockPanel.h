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

private:
    void checkStale();

    QLabel* mMechanism{nullptr};
    QLabel* mCycles{nullptr};
    QLabel* mApproach{nullptr};
    QLabel* mAlert{nullptr};

    QElapsedTimer mLastStatus;
    QTimer mStaleTimer;
};
