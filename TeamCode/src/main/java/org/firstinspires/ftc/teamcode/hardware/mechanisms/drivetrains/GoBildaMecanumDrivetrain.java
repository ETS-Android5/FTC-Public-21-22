package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

public class GoBildaMecanumDrivetrain extends MecanumDrivetrain {
    @Override
    public void driveBySticks(double lateral, double longitudinal, double turn) {
        double wheelPower = Math.hypot(lateral, longitudinal);
        double stickAngleRadians = Math.atan2(longitudinal, lateral) - Math.PI / 4;

        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);

        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

        frontLeftMotorState = frontLeftMotorState.withPower(wheelPower * cosAngleRadians * factor + turn);
        frontRightMotorState = frontRightMotorState.withPower(wheelPower * sinAngleRadians * factor - turn);
        rearLeftMotorState = rearLeftMotorState.withPower(wheelPower * sinAngleRadians * factor + turn);
        rearRightMotorState = rearRightMotorState.withPower(wheelPower * cosAngleRadians * factor - turn);
    }
}
