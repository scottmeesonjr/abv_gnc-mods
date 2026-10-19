#include "abv_gui/PolicySelectorPanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QBrush>

#include "abv_common/RosTopicManager.h"
#include "abv_gui/ButtonAdapter.hpp"

static const QString kLabelStyle = "color: #888888; font-size: 11px;";
static const QString kSuccessStyle = "color: #4caf50; font-size: 11px;";
static const QString kFailureStyle = "color: #e74c3c; font-size: 11px;";

PolicySelectorPanel::PolicySelectorPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(6);

    auto* header = new QLabel("Control Policy");
    header->setStyleSheet("color: #ffffff; font-weight: bold;");
    layout->addWidget(header);

    auto* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    auto* selectorRow = new QHBoxLayout();
    mSelector = new QComboBox();
    selectorRow->addWidget(mSelector);

    auto* switchBtn = new ButtonAdapter("Switch",
            std::bind(&PolicySelectorPanel::onSwitchClicked, this),
            ButtonStyle::primary());
    selectorRow->addWidget(switchBtn);
    layout->addLayout(selectorRow);

    mStatusLabel = new QLabel();
    mStatusLabel->setStyleSheet(kLabelStyle);
    layout->addWidget(mStatusLabel);

    mSwitchClient = RosTopicManager::getInstance()->create_client<abv_msgs::srv::AbvPolicySwitch>("abv/policy/switch");
}

void PolicySelectorPanel::onRegistryUpdate(const QVariant& aData)
{
    QString previousSelection = mSelector->currentText();
    QStringList parts = aData.toString().split(';', Qt::SkipEmptyParts);

    mSelector->blockSignals(true);
    mSelector->clear();

    for (const QString& part : parts)
    {
        QStringList kv = part.split(':');
        if (kv.size() != 2)
        {
            continue;
        }

        mSelector->addItem(kv[0]);
        if (kv[1] != "1")
        {
            mSelector->setItemData(mSelector->count() - 1, QBrush(Qt::gray), Qt::ForegroundRole);
        }
    }

    int previousIndex = mSelector->findText(previousSelection);
    if (previousIndex >= 0)
    {
        mSelector->setCurrentIndex(previousIndex);
    }

    mSelector->blockSignals(false);
}

void PolicySelectorPanel::onSwitchClicked()
{
    if (!mSwitchClient || mSelector->currentText().isEmpty())
    {
        return;
    }

    auto request = std::make_shared<abv_msgs::srv::AbvPolicySwitch::Request>();
    request->policy_name = mSelector->currentText().toStdString();

    mStatusLabel->setStyleSheet(kLabelStyle);
    mStatusLabel->setText("Switching...");

    mSwitchClient->async_send_request(request,
        [this](rclcpp::Client<abv_msgs::srv::AbvPolicySwitch>::SharedFuture aFuture)
        {
            auto response = aFuture.get();
            bool success = response->success;
            QString message = QString::fromStdString(response->message);

            // async_send_request's callback fires on the ROS spin thread, so
            // hop to the GUI thread before touching any widgets - same
            // concern TopicAdapter solves for subscriptions.
            QMetaObject::invokeMethod(this, [this, success, message]()
            {
                mStatusLabel->setText(message);
                mStatusLabel->setStyleSheet(success ? kSuccessStyle : kFailureStyle);
            }, Qt::QueuedConnection);
        });
}
