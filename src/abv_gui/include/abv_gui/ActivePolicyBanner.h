#pragma once

#include <QWidget>
#include <QLabel>
#include <QVariant>

// Prominent, top-center readout of the currently active control policy,
// fed from abv/policy/active (see PolicySelectorPanel for the switching UI,
// which stays in the sidebar - this widget is display-only).
class ActivePolicyBanner : public QWidget
{
    Q_OBJECT
public:
    explicit ActivePolicyBanner(QWidget* parent = nullptr);

public slots:
    // aData is the active policy name (empty string = none active)
    void setActivePolicy(const QVariant& aData);

private:
    QLabel* mLabel{nullptr};
};
