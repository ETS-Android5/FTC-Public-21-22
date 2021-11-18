package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "WTF")
public class Wtf extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FRONT_LEFT_MOTOR");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRONT_RIGHT_MOTOR");
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    DcMotor rearLeftMotor = hardwareMap.dcMotor.get("REAR_LEFT_MOTOR");
    DcMotor rearRightMotor = hardwareMap.dcMotor.get("REAR_RIGHT_MOTOR");
    rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    frontLeftMotor.setPower(.25);
    frontRightMotor.setPower(.25);
    rearLeftMotor.setPower(-.25);
    rearRightMotor.setPower(-.25);
    sleep(4270);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    rearLeftMotor.setPower(0);
    rearRightMotor.setPower(0);
  }
}
