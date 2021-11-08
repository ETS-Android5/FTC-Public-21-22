package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

public interface IMecanumDrivetrain extends IAWDClassicDrivetrain {
    void setFrontLeftPower(double power);
    void setFrontRightPower(double power);
    void setRearLeftPower(double power);
    void setRearRightPower(double power);
    void setFrontLeftTarget(int ticks);
    void setFrontRightTarget(int ticks);
    void setRearLeftTarget(int ticks);
    void setRearRightTarget(int ticks);
    void driveBySticks(double lateral, double longitudinal, double turn);
}
