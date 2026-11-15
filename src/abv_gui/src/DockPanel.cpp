#include "abv_gui/DockPanel.h"

#include <QFormLayout>
#include <QMessageBox>
#include <QStringList>
#include <QFrame>
#include <QVBoxLayout>
#include <QVector>

#include "abv_msgs/msg/dock_status.hpp"

namespace
{
    const QString kFieldStyle = "color: #888888; font-size: 11px;";
    const QString kValueBase =
        "background-color: #2a2a2a;"
        "border: 1px solid #444444;"
        "border-radius: 4px;"
        "padding: 1px 6px;"
        "font-family: monospace;"
        "font-size: 13px;";

    QString valueStyle(const QString& aColor)
    {
        return kValueBase + "color: " + aColor + ";";
    }

    QLabel* makeValueLabel()
    {
        auto* label = new QLabel("---");
        label->setStyleSheet(valueStyle("#ffffff"));
        return label;
    }

    QString stateName(int aState)
    {
        switch (aState)
        {
            case abv_msgs::msg::DockStatus::STATE_IDLE:    return "UNLOCKED";
            case abv_msgs::msg::DockStatus::STATE_ARMED:   return "ARMED";
            case abv_msgs::msg::DockStatus::STATE_CAPTURE: return "CAPTURE";
            case abv_msgs::msg::DockStatus::STATE_LOCKED:  return "LOCKED";
            case abv_msgs::msg::DockStatus::STATE_RELEASE: return "RELEASE";
            case abv_msgs::msg::DockStatus::STATE_FAULT:   return "FAULT";
            default:                                       return "?";
        }
    }

    // "APPROACH" -> "Approach", "BACK OFF" -> "Back off"
    QString niceLabel(const QString& aLabel)
    {
        QString s = aLabel.toLower();
        if (!s.isEmpty()) s[0] = s[0].toUpper();
        return s;
    }

    // s, mechanism shown as "NO DATA" if no status arrives within this time
    constexpr int kStaleMs = 1000;
}

DockPanel::DockPanel(QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    auto* header = new QLabel("Docking");
    header->setStyleSheet("color: #cccccc; font-weight: bold; font-size: 14px;");
    layout->addWidget(header);

    auto* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setStyleSheet("color: #444444;");
    layout->addWidget(line);

    auto* form = new QFormLayout();
    form->setSpacing(4);
    form->setContentsMargins(4, 0, 0, 0);

    mMechanism = makeValueLabel();
    mCycles = makeValueLabel();
    mApproach = makeValueLabel();
    mLastCycle = makeValueLabel();
    mStep = makeValueLabel();
    mRunCycle = makeValueLabel();
    mCaptured = makeValueLabel();

    auto addRow = [form](const QString& aName, QLabel* aValue) {
        auto* name = new QLabel(aName);
        name->setStyleSheet(kFieldStyle);
        form->addRow(name, aValue);
    };
    addRow("Cycle:", mRunCycle);
    addRow("Step:", mStep);
    addRow("Captured:", mCaptured);
    addRow("Mechanism:", mMechanism);
    addRow("Mech. total:", mCycles);
    addRow("Closing / gap:", mApproach);
    addRow("Last cycle:", mLastCycle);
    layout->addLayout(form);

    mAlert = new QLabel("Speed: ---");
    mAlert->setAlignment(Qt::AlignCenter);
    mAlert->setMinimumHeight(26);
    mAlert->setStyleSheet("background-color: #444444; color: white; font-weight: bold; border-radius: 13px;");
    layout->addWidget(mAlert);

    mLastStatus.invalidate();
    mStaleTimer.setInterval(500);
    connect(&mStaleTimer, &QTimer::timeout, this, &DockPanel::checkStale);
    mStaleTimer.start();
}

void DockPanel::onDockStatus(const QVariant& aData)
{
    QVector<double> s = aData.value<QVector<double>>();
    if (s.size() != 6)
    {
        return;
    }
    mLastStatus.restart();

    const bool locked = s[1] > 0.5;
    const bool linkOk = s[2] > 0.5;
    const bool estop = s[3] > 0.5;
    const int fault = static_cast<int>(s[5]);

    QString text = stateName(static_cast<int>(s[0]));
    QString color = "#ffffff";
    if (!linkOk)
    {
        text = "LINK DOWN";
        color = "#ff5252";
    }
    else if (estop)
    {
        text = "E-STOP";
        color = "#ff5252";
    }
    else if (fault != 0)
    {
        text = QString("FAULT %1").arg(fault);
        color = "#ff5252";
    }
    else if (locked)
    {
        color = "#66bb6a";
    }
    else if (s[0] == abv_msgs::msg::DockStatus::STATE_ARMED)
    {
        color = "#ffb300";
    }
    mMechanism->setText(text);
    mMechanism->setStyleSheet(valueStyle(color));
    mCycles->setText(QString::number(static_cast<int>(s[4])));
}

void DockPanel::onNavAlert(const QVariant& aData)
{
    QVector<double> a = aData.value<QVector<double>>();
    if (a.size() != 4)
    {
        return;
    }
    mApproach->setText(QString("%1 m/s / %2 m").arg(a[1], 0, 'f', 3).arg(a[2], 0, 'f', 3));

    if (a[3] > 0.5)
    {
        mAlert->setText(QString("OVERSPEED %1 m/s").arg(a[0], 0, 'f', 3));
        mAlert->setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; border-radius: 13px;");
    }
    else
    {
        mAlert->setText(QString("Speed OK %1 m/s").arg(a[0], 0, 'f', 3));
        mAlert->setStyleSheet("background-color: #4caf50; color: white; font-weight: bold; border-radius: 13px;");
    }
}

void DockPanel::checkStale()
{
    if (!mLastStatus.isValid() || mLastStatus.elapsed() > kStaleMs)
    {
        mMechanism->setText("NO DATA");
        mMechanism->setStyleSheet(valueStyle("#888888"));
    }
}

void DockPanel::onDockEvent(const QVariant& aData)
{
    const QStringList parts = aData.toString().split('|');
    if (parts.isEmpty())
    {
        return;
    }

    if (parts[0] == "STEP" && parts.size() >= 5)
    {
        // STEP|<cycle>|<total>|<step number, 0 = end>|<label>
        const int cycle = parts[1].toInt();
        const int total = parts[2].toInt();
        const int stepNum = parts[3].toInt();
        const QString label = parts[4];

        if (cycle == 1 && stepNum == 1)
        {
            // first step of a new run: reset the run tally
            mRunAttempted = 0;
            mRunCaptured = 0;
            mCaptured->setText("0 of 0");
            mCaptured->setStyleSheet(valueStyle("#ffffff"));
            mLastCycle->setText("---");
            mLastCycle->setStyleSheet(valueStyle("#ffffff"));
        }

        mRunCycle->setText(QString("%1 of %2").arg(cycle).arg(total));
        mStep->setText(stepNum > 0 ? QString("%1/7  %2").arg(stepNum).arg(niceLabel(label))
                                   : QString("%1 (end)").arg(niceLabel(label)));
        QString color = "#ffffff";
        if (label == "APPROACH")     color = "#ffb300";
        else if (label == "LOCK")    color = "#66bb6a";
        else if (label == "UNLOCK")  color = "#4fc3f7";
        mStep->setStyleSheet(valueStyle(color));
        return;
    }

    if (parts[0] == "CYCLE" && parts.size() >= 4)
    {
        const bool captured = (parts[2] == "CAPTURED");
        mRunAttempted++;
        if (captured) mRunCaptured++;
        mCaptured->setText(QString("%1 of %2").arg(mRunCaptured).arg(mRunAttempted));
        mCaptured->setStyleSheet(valueStyle(mRunCaptured == mRunAttempted ? "#66bb6a" : "#ff5252"));
        mLastCycle->setText(QString("#%1 %2 %3 m/s")
                                .arg(parts[1], parts[2])
                                .arg(parts[3].toDouble(), 0, 'f', 3));
        mLastCycle->setStyleSheet(valueStyle(captured ? "#66bb6a" : "#ff5252"));
        return;
    }

    QString title;
    QString text;
    QMessageBox::Icon icon = QMessageBox::Information;
    if (parts[0] == "COMPLETE" && parts.size() >= 3)
    {
        title = "Docking run complete";
        mStep->setText("COMPLETE");
        mStep->setStyleSheet(valueStyle("#66bb6a"));
        text = QString("Docking path complete.\n\n%1 of %2 cycles captured.\n\n"
                       "Results are in ~/abv_dock_logs/.").arg(parts[1], parts[2]);
        if (parts[1] != parts[2])
        {
            icon = QMessageBox::Warning;
        }
    }
    else if (parts[0] == "ABORTED")
    {
        mStep->setText("ABORTED");
        mStep->setStyleSheet(valueStyle("#ff5252"));
        title = "Docking run aborted";
        text = "Docking run aborted: " + parts.mid(1).join('|');
        icon = QMessageBox::Warning;
    }
    else
    {
        return;
    }

    // non-modal so the GUI keeps updating while the box is open
    auto* box = new QMessageBox(icon, title, text, QMessageBox::Ok, this);
    box->setAttribute(Qt::WA_DeleteOnClose);
    box->setModal(false);
    box->show();
}
