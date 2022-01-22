package org.firstinspires.ftc.teamcode.opmodes.info;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "DrivetrainWiring")
@Disabled
public class DrivetrainWiring extends LinearOpMode {
  @Override
  public void runOpMode() {
    DcMotor fl = hardwareMap.dcMotor.get("FRONT_LEFT_MOTOR");
    DcMotor fr = hardwareMap.dcMotor.get("FRONT_RIGHT_MOTOR");
    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    DcMotor rl = hardwareMap.dcMotor.get("REAR_LEFT_MOTOR");
    DcMotor rr = hardwareMap.dcMotor.get("REAR_RIGHT_MOTOR");
    rl.setDirection(DcMotorSimple.Direction.REVERSE);

    DcMotor[] motors = new DcMotor[] {fl, fr, rl, rr};

    for (DcMotor motor : motors) {
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    waitForStart();

    for (DcMotor motor : motors) {
      motor.setPower(1);
      sleep(1000);
      motor.setPower(0);
      telemetry.addData("FL", fl.getCurrentPosition());
      telemetry.addData("FR", fr.getCurrentPosition());
      telemetry.addData("RL", rl.getCurrentPosition());
      telemetry.addData("RR", rr.getCurrentPosition());
      telemetry.update();
      sleep(5000);
    }
  }
}
