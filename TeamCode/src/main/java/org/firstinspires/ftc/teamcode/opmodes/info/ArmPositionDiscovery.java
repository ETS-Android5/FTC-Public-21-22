package org.firstinspires.ftc.teamcode.opmodes.info;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmPositionDiscovery")
@Disabled
@SuppressWarnings("unused")
public class ArmPositionDiscovery extends OpMode {
  private DcMotorEx fl;
  private DcMotorEx fr;
  private DcMotorEx rl;
  private DcMotorEx rr;

  private DcMotorEx turret;
  private DcMotorEx firstJoint;
  private DcMotorEx intake;
  private Servo secondJoint;
  private Servo gripper;

  private double secondJointPosition = 0.5;
  private double gripperPosition = 0.5;

  private double intakeSpeed = 0;

  private double turretSpeed = 0;

  @Override
  public void init() {
    fl = hardwareMap.get(DcMotorEx.class, "FRONT_LEFT_MOTOR");
    fr = hardwareMap.get(DcMotorEx.class, "FRONT_RIGHT_MOTOR");
    rl = hardwareMap.get(DcMotorEx.class, "REAR_LEFT_MOTOR");
    rr = hardwareMap.get(DcMotorEx.class, "REAR_RIGHT_MOTOR");

    turret = hardwareMap.get(DcMotorEx.class, "TURRET_MOTOR");
    firstJoint = hardwareMap.get(DcMotorEx.class, "LIFT_JOINT_ONE_MOTOR");
    intake = hardwareMap.get(DcMotorEx.class, "INTAKE_MOTOR");
    secondJoint = hardwareMap.servo.get("LIFT_JOINT_TWO_SERVO");
    gripper = hardwareMap.servo.get("SINGLE_SERVO_GRIPPER_SERVO");

    reset(fl);
    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    reset(fr);
    reset(rl);
    rl.setDirection(DcMotorSimple.Direction.REVERSE);
    reset(rr);

    reset(turret);
    reset(firstJoint);
    firstJoint.setDirection(DcMotorSimple.Direction.REVERSE);
    reset(intake);
    intake.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  private void reset(DcMotorEx motor) {
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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

    turretSpeed -= gamepad1.left_trigger / 1000;
    turretSpeed += gamepad1.right_trigger / 1000;
    turret.setPower(turretSpeed);

    telemetry.addData("FL", fl.getCurrentPosition());
    telemetry.addData("FR", fr.getCurrentPosition());
    telemetry.addData("RL", rl.getCurrentPosition());
    telemetry.addData("RR", rr.getCurrentPosition());

    telemetry.addData("TURRET", turret.getCurrentPosition());
    telemetry.addData("TURRET_VIA_INTAKE", intake.getCurrentPosition());
    telemetry.addData("FIRST JOINT", firstJoint.getCurrentPosition());
    telemetry.addData("INTAKE SPEED", intakeSpeed);
    telemetry.addData("SECOND JOINT", secondJointPosition);
    telemetry.addData("GRIPPER", gripperPosition);
  }
}
