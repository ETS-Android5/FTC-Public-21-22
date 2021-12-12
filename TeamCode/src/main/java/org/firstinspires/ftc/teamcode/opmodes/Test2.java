package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test2")
public class Test2 extends OpMode {
  private DcMotor turret;
  private DcMotor firstJoint;
  private DcMotor intake;
  private Servo secondJoint;
  private Servo gripper;

  private double secondJointPosition = 0.5;
  private double gripperPosition = 0.5;

  private double intakeSpeed = 0;

  @Override
  public void init() {
    turret = hardwareMap.dcMotor.get("TURRET_MOTOR");
    firstJoint = hardwareMap.dcMotor.get("LIFT_JOINT_ONE_MOTOR");
    intake = hardwareMap.dcMotor.get("INTAKE_MOTOR");
    secondJoint = hardwareMap.servo.get("LIFT_JOINT_TWO_SERVO");
    gripper = hardwareMap.servo.get("SINGLE_SERVO_GRIPPER_SERVO");
    reset(turret);
    reset(firstJoint);
    reset(intake);
  }

  private void reset(DcMotor motor) {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motor.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  @Override
  public void loop() {
    secondJointPosition += gamepad1.left_stick_y * .001;
    gripperPosition += gamepad1.right_stick_y * .001;
    secondJointPosition =
        secondJointPosition < 0 ? 0 : secondJointPosition > 1 ? 1 : secondJointPosition;
    gripperPosition = gripperPosition < 0 ? 0 : gripperPosition > 1 ? 1 : gripperPosition;
    secondJoint.setPosition(secondJointPosition);
    gripper.setPosition(gripperPosition);
    if (gamepad1.x) {
      intakeSpeed = 1;
    }
    if (gamepad1.b) {
      intakeSpeed = -1;
    }
    if (gamepad1.a) {
      intakeSpeed = 0;
    }
    intake.setPower(intakeSpeed);
    telemetry.addData("TURRET", turret.getCurrentPosition());
    telemetry.addData("FIRST JOINT", firstJoint.getCurrentPosition());
    telemetry.addData("INTAKE SPEED", intakeSpeed);
    telemetry.addData("SECOND JOINT", secondJointPosition);
    telemetry.addData("GRIPPER", gripperPosition);
  }
}
