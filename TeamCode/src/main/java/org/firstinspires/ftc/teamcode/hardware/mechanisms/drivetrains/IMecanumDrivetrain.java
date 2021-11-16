package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

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
    void setPowerCurve(TriFunction<Integer, Integer, Integer, Double> powerCurve);
}
