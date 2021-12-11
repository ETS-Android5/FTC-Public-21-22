package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test2")
public class Test2 extends OpMode {
  private DcMotor turret;
  private DcMotor firstJoint;
  private Servo secondJoint;
  private Servo gripper;

  private double secondJointPosition = 0;
  private double gripperPosition = 0;

  @Override
  public void init() {
    turret = hardwareMap.dcMotor.get("TURRET_MOTOR");
    firstJoint = hardwareMap.dcMotor.get("LIFT_JOINT_ONE_MOTOR");
    secondJoint = hardwareMap.servo.get("LIFT_JOINT_TWO_SERVO");
    gripper = hardwareMap.servo.get("SINGLE_SERVO_GRIPPER_SERVO");
    reset(turret);
    reset(firstJoint);
  }

  private void reset(DcMotor motor) {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }

  @Override
  public void loop() {
    secondJointPosition += gamepad1.left_stick_y * .1;
    gripperPosition += gamepad1.right_stick_y * .1;
    secondJoint.setPosition(secondJointPosition);
    gripper.setPosition(gripperPosition);
    telemetry.addData("TURRET", turret.getCurrentPosition());
    telemetry.addData("FIRST JOINT", firstJoint.getCurrentPosition());
    telemetry.addData("SECOND JOINT", secondJointPosition);
    telemetry.addData("GRIPPER", gripperPosition);
  }
}
