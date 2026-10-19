#pragma once

#include <QWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QStackedWidget>
#include <QCheckBox>
#include <QVariant>
#include <QVector>

#include <rclcpp/rclcpp.hpp>

#include "abv_msgs/msg/abv_controller_command.hpp"

class CommandPanel : public QWidget
{
    Q_OBJECT
public:
    explicit CommandPanel(QWidget* parent = nullptr);

    // Mirrors the live vehicle pose into the Pose panel's X/Y/Yaw fields, so
    // they start from the current pose instead of 0. Skips any field the
    // user currently has focused or has edited since the last send (see
    // mPoseXDirty et al.) so it doesn't clobber an in-progress/pending edit.
    void setCurrentPose(double aX, double aY, double aYaw);

    // Builds and publishes an AbvGuidanceCommand for the given pose - type
    // "line" (no obstacles known) or "avoid" (obstacles present, see
    // setObstaclesPresent) - since only CollisionAvoidPathGenerator actually
    // checks for collisions. Factored out of onSendPose() so other callers
    // (e.g. the table view's click-to-set-goal) can send a pose without
    // going through the Pose panel's spin boxes.
    void sendPoseCommand(double aX, double aY, double aYaw);

    // Called by main.cpp whenever the published obstacle set (abv/scene/obstacles)
    // changes, so sendPoseCommand() knows whether to route through
    // CollisionAvoidPathGenerator ("avoid") or the plain "line" generator.
    void setObstaclesPresent(bool aPresent);

public slots:
    // Slot counterpart of setCurrentPose for TopicAdapter::newDataVariant.
    // Connecting a signal to this (rather than calling setCurrentPose
    // directly from the converter) ensures the widget update runs on the
    // GUI thread instead of racing the ROS subscription thread with the
    // user's typing/click on the same spin boxes.
    void onPoseSync(const QVariant& aData);

private:
    QWidget* makePosePanel();
    QWidget* makeVelocityPanel();
    QWidget* makePathPanel();
    QWidget* makeThrusterPanel();  

    QWidget* makeThrusterButton(const std::string& name, int number);
    QWidget* makeAxisButton(const std::string& name, int axis);
    abv_msgs::msg::AbvControllerCommand makeCommand(int axis);
    abv_msgs::msg::AbvControllerCommand makeThruster(int thruster);

    void onSendPose();
    void onSendVelocity();
    void onSendPath();

    QDoubleSpinBox* mPoseX{nullptr};
    QDoubleSpinBox* mPoseY{nullptr};
    QDoubleSpinBox* mPoseYaw{nullptr};

    // Set when the user edits the corresponding pose field, so a live pose
    // sync arriving between mouse-press and mouse-release on "Send Pose"
    // (the spin box already lost focus to the button by mouse-press, before
    // clicked() fires on release) can't overwrite the pending edit. Cleared
    // once the edit is sent, so the field resumes tracking live pose.
    bool mPoseXDirty{false};
    bool mPoseYDirty{false};
    bool mPoseYawDirty{false};

    QDoubleSpinBox* mVelX{nullptr};
    QDoubleSpinBox* mVelY{nullptr};
    QDoubleSpinBox* mVelYaw{nullptr};

    QComboBox*      mPathType{nullptr};
    QDoubleSpinBox* mPathDuration{nullptr};

    QCheckBox*      mBodyFrameCheckbox{nullptr};

    rclcpp::TimerBase::SharedPtr mFireTimer{nullptr};

    bool mObstaclesPresent{false};
};