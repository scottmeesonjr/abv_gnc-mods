#include "abv_gui/ActivePolicyBanner.h"
#include <QHBoxLayout>

// Window background is rgb(30,30,30) and most panels sit on #2a2a2a - these
// use a distinct blue-slate hue so the banner reads as its own strip rather
// than blending in, full width across the top of the window.
static const QString kActiveStyle =
    "color: #ffffff;"
    "background-color: #1b3a2a;"
    "border-bottom: 2px solid #4caf50;"
    "padding: 8px 0px;"
    "font-weight: bold;"
    "font-size: 16px;";

// no policy active means nothing is driving the vehicle in POSE/VELOCITY
// control - worth a soft red warning tint rather than a neutral color
static const QString kInactiveStyle =
    "color: #e8b4b4;"
    "background-color: #3a2020;"
    "border-bottom: 2px solid #a85c5c;"
    "padding: 8px 0px;"
    "font-weight: bold;"
    "font-size: 16px;";

ActivePolicyBanner::ActivePolicyBanner(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    mLabel = new QLabel("No Active Policy");
    mLabel->setAlignment(Qt::AlignCenter);
    mLabel->setStyleSheet(kInactiveStyle);

    layout->addWidget(mLabel);
}

void ActivePolicyBanner::setActivePolicy(const QVariant& aData)
{
    QString activePolicy = aData.toString();

    if (activePolicy.isEmpty())
    {
        mLabel->setText("No Active Policy");
        mLabel->setStyleSheet(kInactiveStyle);
    }
    else
    {
        mLabel->setText("Active Policy: " + activePolicy.toUpper());
        mLabel->setStyleSheet(kActiveStyle);
    }
}
