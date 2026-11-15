#include "abv_gui/CommandPanel.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QStackedWidget>
#include <QFrame>
#include <QSignalBlocker>
#include <cmath>

#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_guidance_command.hpp"
#include "std_msgs/msg/string.hpp"
#include <QLineEdit>
#include <QSpinBox>

#include "abv_gui/ButtonAdapter.hpp"

static QDoubleSpinBox* makeSpinBox(double min, double max, double step = 0.05)
{
    auto* sb = new QDoubleSpinBox();
    sb->setRange(min, max);
    sb->setSingleStep(step);
    sb->setDecimals(3);
    sb->setValue(0.0);
    return sb;
}

CommandPanel::CommandPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(8);

    // Separator
    auto* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);
    
    // Command type selector
    auto* typeLabel = new QLabel("Command");
    layout->addWidget(typeLabel);

    auto* commandLayout = new QHBoxLayout();

    auto* selector  = new QComboBox();
    selector->addItems({"Pose", "Velocity", "Path", "Thruster"});
    commandLayout->addWidget(selector);

    // Global/Body Frame Checkbox
    mBodyFrameCheckbox = new QCheckBox("Body Frame");
    commandLayout->addWidget(mBodyFrameCheckbox);
    layout->addLayout(commandLayout);

    // Stacked panels
    auto* stack = new QStackedWidget();
    stack->addWidget(makePosePanel());
    stack->addWidget(makeVelocityPanel());
    stack->addWidget(makePathPanel()); 
    stack->addWidget(makeThrusterPanel()); 
    layout->addWidget(stack);

    connect(selector, &QComboBox::currentIndexChanged,
            stack,    &QStackedWidget::setCurrentIndex);
}

QWidget* CommandPanel::makePosePanel()
{
    auto* panel  = new QWidget();
    auto* layout = new QVBoxLayout(panel);
    auto* form   = new QFormLayout();

    mPoseX = makeSpinBox(-100.0, 100.0);
    mPoseY = makeSpinBox(-100.0, 100.0);
    mPoseYaw = makeSpinBox(-100.0, 100.0);

    // only real user edits should mark a field dirty; setCurrentPose()
    // blocks signals around its own setValue() calls so those don't loop
    // back and immediately mark the field dirty again
    connect(mPoseX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { mPoseXDirty = true; });
    connect(mPoseY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { mPoseYDirty = true; });
    connect(mPoseYaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { mPoseYawDirty = true; });

    form->addRow("X:", mPoseX);
    form->addRow("Y:", mPoseY);
    form->addRow("Yaw:", mPoseYaw);

    auto* sendBtn = new ButtonAdapter("Send Pose", 
            std::bind(&CommandPanel::onSendPose, this), 
            ButtonStyle::primary());
    sendBtn->button()->setMinimumHeight(50); 

    layout->addLayout(form);
    layout->addWidget(sendBtn);
    layout->addStretch(); 
    return panel;
}

QWidget* CommandPanel::makeVelocityPanel()
{
    auto* panel  = new QWidget();
    auto* layout = new QVBoxLayout(panel);
    auto* form   = new QFormLayout();

    mVelX = makeSpinBox(-10.0, 10.0);
    mVelY = makeSpinBox(-10.0, 10.0);
    mVelYaw = makeSpinBox(-10.0, 10.0);

    form->addRow("X:", mVelX);
    form->addRow("Y:", mVelY);
    form->addRow("Yaw:", mVelYaw);

    auto* sendBtn = new ButtonAdapter("Send Velocity", 
            std::bind(&CommandPanel::onSendVelocity, this), 
            ButtonStyle::primary());
    sendBtn->button()->setMinimumHeight(50); 

    layout->addLayout(form);
    layout->addWidget(sendBtn);
    layout->addStretch(); 
    return panel;
}

QWidget* CommandPanel::makePathPanel()
{
    auto* panel  = new QWidget();
    auto* layout = new QVBoxLayout(panel);
    auto* form   = new QFormLayout();

    mPathType = new QComboBox();
    mPathType->addItems({"File", "Dock"});
    form->addRow("Path Type:", mPathType);

    // -1 = no overall time limit (guidance skips its path watchdog). The old
    // default of 0-1 s ended paths almost immediately.
    mPathDuration = makeSpinBox(-1.0, 3600.0, 10);
    mPathDuration->setDecimals(0);
    mPathDuration->setSpecialValueText("No limit");
    mPathDuration->setValue(-1.0);
    form->addRow("Duration (s):", mPathDuration); 

    // Dock run test settings
    mDockOptions = new QWidget();
    auto* dockForm = new QFormLayout(mDockOptions);
    dockForm->setContentsMargins(0, 0, 0, 0);

    mDockRadial = makeSpinBox(-0.20, 0.20, 0.01);
    mDockRadial->setToolTip("Sideways offset across the docking axis (SYS.07 limit: 0.10 m)");
    dockForm->addRow("Radial (m):", mDockRadial);

    mDockAngleDeg = makeSpinBox(-10.0, 10.0, 0.5);
    mDockAngleDeg->setDecimals(1);
    mDockAngleDeg->setToolTip("Heading offset from aligned (SYS.08 limit: 4 deg)");
    dockForm->addRow("Angle (deg):", mDockAngleDeg);

    mDockSpeed = makeSpinBox(0.0, 0.15, 0.005);
    mDockSpeed->setSpecialValueText("Config");
    mDockSpeed->setToolTip("Approach speed; SYS.10 band is 0.05-0.10 m/s. 'Config' uses Docking.ApproachVelocity");
    dockForm->addRow("Speed (m/s):", mDockSpeed);

    mDockCycles = new QSpinBox();
    mDockCycles->setRange(0, 50);
    mDockCycles->setValue(0);
    mDockCycles->setSpecialValueText("Config");
    mDockCycles->setToolTip("Cycles in this run (SYS.06: 10). 'Config' uses Docking.Cycles");
    dockForm->addRow("Cycles:", mDockCycles);

    mDockRunName = new QLineEdit();
    mDockRunName->setPlaceholderText("optional, e.g. radial_0p10");
    mDockRunName->setToolTip("Added to this run's log file name in ~/abv_dock_logs/");
    dockForm->addRow("Run name:", mDockRunName);

    mDockCondition = new QLineEdit();
    mDockCondition->setPlaceholderText("optional note, e.g. fixture roll 4 deg");
    mDockCondition->setToolTip("Copied into the 'condition' column of every log row");
    dockForm->addRow("Note:", mDockCondition);

    mDockOptions->setVisible(false);
    // Dock runs don't use Duration: every step has its own time limit
    // (standoff 120 s, approach Docking.ApproachTimeout, fixed holds), so a
    // run can't hang. Duration stays for File paths.
    connect(mPathType, &QComboBox::currentTextChanged, this,
            [this](const QString& aType) {
                const bool dock = (aType == "Dock");
                mDockOptions->setVisible(dock);
                mPathDuration->setEnabled(!dock);
                mPathDuration->setToolTip(dock ? "Not used for Dock runs: each step has its own time limit"
                                               : "Overall time limit for the path");
            });

    auto* sendBtn = new ButtonAdapter("Send Path", 
            std::bind(&CommandPanel::onSendPath, this), 
            ButtonStyle::primary());
    sendBtn->button()->setMinimumHeight(50); 

    layout->addLayout(form);
    layout->addWidget(mDockOptions);
    layout->addWidget(sendBtn);
    layout->addStretch(); 
    return panel;
}

QWidget* CommandPanel::makeThrusterPanel()
{
    auto* panel = new QWidget();
    auto* main  = new QVBoxLayout(panel);
    main->setContentsMargins(0, 0, 0, 0);
    main->setSpacing(8);

    // ── Thruster header ───────────────────────────────────────────────────────
    auto* thrusterHeader = new QLabel("Thrusters");
    thrusterHeader->setStyleSheet("color: #cccccc; font-weight: bold; font-size: 12px;");
    main->addWidget(thrusterHeader);

    auto* thrusterLine = new QFrame();
    thrusterLine->setFrameShape(QFrame::HLine);
    thrusterLine->setStyleSheet("color: #444444;");
    main->addWidget(thrusterLine);

    // ── Thruster grid ─────────────────────────────────────────────────────────
    auto* grid = new QGridLayout();
    grid->setSpacing(4);
    grid->setContentsMargins(0, 0, 0, 0);

    for (int i = 1; i <= 8; i++) {
        std::string name = "T" + std::to_string(i);
        auto* btn = makeThrusterButton(name, i);
        grid->addWidget(btn, (i - 1) / 4, (i - 1) % 4);
    }
    main->addLayout(grid);

    // ── Axes header ───────────────────────────────────────────────────────────
    auto* axesHeader = new QLabel("Axes");
    axesHeader->setStyleSheet("color: #cccccc; font-weight: bold; font-size: 12px;");
    main->addWidget(axesHeader);

    auto* axesLine = new QFrame();
    axesLine->setFrameShape(QFrame::HLine);
    axesLine->setStyleSheet("color: #444444;");
    main->addWidget(axesLine);

    // ── Axes grid ─────────────────────────────────────────────────────────────
    auto* axesGrid = new QGridLayout();
    axesGrid->setSpacing(4);
    axesGrid->setContentsMargins(0, 0, 0, 0);

    const QList<std::string> axisNames = {"+X", "-X", "+Y", "-Y", "+Yaw", "-Yaw"};
    for (int i = 0; i < axisNames.size(); i++) {
    
        auto* btn = makeAxisButton(axisNames[i], i);
        axesGrid->addWidget(btn, i / 3, i % 3);  // 2 rows of 3
    }
    main->addLayout(axesGrid);

    main->addStretch();
    return panel;
}

abv_msgs::msg::AbvControllerCommand CommandPanel::makeCommand(int axis)
{
    // assumes axis order of +x, -x, +y, -y, +yaw, -yaw
    abv_msgs::msg::AbvControllerCommand cmd;
    cmd.set__type("direction");
    
    abv_msgs::msg::AbvVec3 msg;

    switch (axis)
    {
    case 0: // +x
        msg.x = 1;  
        break;
    case 1: 
        msg.x = -1; 
        break; 
    case 2: 
        msg.y = 1; 
        break; 
    case 3: 
        msg.y = -1; 
        break; 
    case 4: 
        msg.yaw = 1; 
        break; 
    case 5: 
        msg.yaw = -1; 
        break; 
    default:
        msg.x = 0; 
        msg.y = 0; 
        msg.yaw = 0; 
        break;      
    }
    
    cmd.set__data(msg); 
    cmd.is_global = !mBodyFrameCheckbox->isChecked();
    return cmd; 
}

abv_msgs::msg::AbvControllerCommand CommandPanel::makeThruster(int thruster)
{
    abv_msgs::msg::AbvControllerCommand cmd;
    cmd.set__type("thruster");
    
    std::string seq = "00000000"; 
    if(thruster != -1)
        seq[thruster-1] = '1'; 
    
    cmd.set__thrusters(seq); 
    return cmd;
}

QWidget* CommandPanel::makeThrusterButton(const std::string& name, int number)
{
    auto* btn = new ButtonAdapter(
        name,
        [this, number]() { 
        mFireTimer = RosTopicManager::getInstance()->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            [this, number]() {
                RosTopicManager::getInstance()->publishMessage(
                    "abv/controller/command", makeThruster(number));
            });
    },  // pressed — starts timer,
        [this]() {
            if (mFireTimer) {
                mFireTimer->cancel();
                mFireTimer = nullptr;
            }
            RosTopicManager::getInstance()->publishMessage(
                "abv/controller/command", makeThruster(-1));  // zero command on release
        },  // released — stops timer
        ButtonStyle::warning()
    );

    return btn; 
}

QWidget* CommandPanel::makeAxisButton(const std::string& name, int axis)
{
    auto* btn = new ButtonAdapter(
        name,
        [this, axis]() { 
        mFireTimer = RosTopicManager::getInstance()->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            [this, axis]() {
                RosTopicManager::getInstance()->publishMessage(
                    "abv/controller/command", makeCommand(axis));
            });
    },  // pressed — starts timer,
        [this]() {
            if (mFireTimer) {
                mFireTimer->cancel();
                mFireTimer = nullptr;
            }
            RosTopicManager::getInstance()->publishMessage(
                "abv/controller/command", makeCommand(-1));  // zero command on release
        },  // released — stops timer
        ButtonStyle::warning()
    );

    return btn;
}

void CommandPanel::onPoseSync(const QVariant& aData)
{
    QVector<double> pose = aData.value<QVector<double>>();
    if (pose.size() != 3)
    {
        return;
    }

    setCurrentPose(pose[0], pose[1], pose[2]);
}

void CommandPanel::setCurrentPose(double aX, double aY, double aYaw)
{
    if (mPoseX && !mPoseX->hasFocus() && !mPoseXDirty)
    {
        QSignalBlocker blocker(mPoseX);
        mPoseX->setValue(aX);
    }
    if (mPoseY && !mPoseY->hasFocus() && !mPoseYDirty)
    {
        QSignalBlocker blocker(mPoseY);
        mPoseY->setValue(aY);
    }
    if (mPoseYaw && !mPoseYaw->hasFocus() && !mPoseYawDirty)
    {
        QSignalBlocker blocker(mPoseYaw);
        mPoseYaw->setValue(aYaw);
    }
}

void CommandPanel::onSendPose()
{
    sendPoseCommand(mPoseX->value(), mPoseY->value(), mPoseYaw->value());

    // sent - resume tracking live pose until the user edits a field again
    mPoseXDirty = false;
    mPoseYDirty = false;
    mPoseYawDirty = false;
}

void CommandPanel::sendPoseCommand(double aX, double aY, double aYaw)
{
    abv_msgs::msg::AbvVec3 pose;
    pose.set__x(aX);
    pose.set__y(aY);
    pose.set__yaw(aYaw);

    abv_msgs::msg::AbvState goalState;
    goalState.set__position(pose);

    abv_msgs::msg::AbvGuidanceCommand cmd;
    cmd.set__type(mObstaclesPresent ? "avoid" : "line");
    cmd.set__duration(-1);
    cmd.set__goal_state(goalState);

    RosTopicManager::getInstance()->publishMessage<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command", cmd);
}

void CommandPanel::setObstaclesPresent(bool aPresent)
{
    mObstaclesPresent = aPresent;
}

void CommandPanel::onSendVelocity()
{
    abv_msgs::msg::AbvVec3 vel; 
    vel.set__x(mVelX->value()); 
    vel.set__y(mVelY->value()); 
    vel.set__yaw(mVelYaw->value()); 

    abv_msgs::msg::AbvControllerCommand cmd; 
    cmd.set__type("vel");
    cmd.set__data(vel); 

    cmd.is_global = !mBodyFrameCheckbox->isChecked();
    
    RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd);
}

void CommandPanel::onSendPath()
{
    abv_msgs::msg::AbvGuidanceCommand cmd; 
    cmd.set__type(mPathType->currentText().toStdString()); 
    cmd.set__duration(mPathDuration->value()); 

    if (mPathType->currentText() == "Dock")
    {
        // name first, so the logger has it before the run's first ARM
        std_msgs::msg::String name;
        name.data = mDockRunName->text().trimmed().toStdString();
        RosTopicManager::getInstance()->publishMessage<std_msgs::msg::String>("abv/test/run_name", name);

        std_msgs::msg::String note;
        note.data = mDockCondition->text().trimmed().toStdString();
        RosTopicManager::getInstance()->publishMessage<std_msgs::msg::String>("abv/test/condition", note);

        cmd.set__duration(-1.0);

        abv_msgs::msg::AbvVec3 goal;
        goal.set__x(mDockRadial->value());
        goal.set__y(mDockSpeed->value());  // 0 = use config
        goal.set__yaw(mDockAngleDeg->value() * M_PI / 180.0);

        abv_msgs::msg::AbvVec3 extra;
        extra.set__x(static_cast<double>(mDockCycles->value()));  // cycles, 0 = config

        abv_msgs::msg::AbvState goalState;
        goalState.set__position(goal);
        goalState.set__velocity(extra);
        cmd.set__goal_state(goalState);
    }

    RosTopicManager::getInstance()->publishMessage("abv/guidance/command", cmd); 
}