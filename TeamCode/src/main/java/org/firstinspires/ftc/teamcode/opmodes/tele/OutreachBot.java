package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OutreachBot extends LinearOpMode {

  @Override
  public void runOpMode() {

    ElapsedTime buttonTimer = new ElapsedTime();
    buttonTimer.reset();

    double buttonTimerCurrentLoopTime;
    double buttonTimerPrevLoopTimeRL = 0;
    double buttonTimerPrevLoopTimeUD = 0;

    // RL is Right/Left servo.
    double tapeMeasureRLServoPosition =
        .5; // This is init position and tracking of current position
    double tapeMeasureRLServoTickMS =
        50; // This is milliseconds between allowed movements of each tick
    double tapeMeasureRLServoTick = 1.0 / 200.0; // This is amount of each tick

    // UD is Up/Down servo.
    double tapeMeasureUDServoPosition =
        .5; // This is init position and tracking of current position
    double tapeMeasureUDServoTickMS =
        150; // This is milliseconds between allowed movements of each tick
    double tapeMeasureUDServoTick = 1.0 / 100.0; // This is amount of each tick

    Servo rlServo = hardwareMap.servo.get("TAPE_MEASURE_YAW_SERVO");
    Servo udServo = hardwareMap.servo.get("TAPE_MEASURE_PITCH_SERVO");
    CRServo extensionServo = hardwareMap.crservo.get("TAPE_MEASURE_LENGTH_SERVO");

    // init RL and UD servos
    rlServo.setPosition(tapeMeasureRLServoPosition);
    udServo.setPosition(tapeMeasureUDServoPosition);

    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FRONT_LEFT_MOTOR");
    DcMotor motorBackLeft = hardwareMap.dcMotor.get("REAR_LEFT_MOTOR");
    DcMotor motorFrontRight = hardwareMap.dcMotor.get("FRONT_RIGHT_MOTOR");
    DcMotor motorBackRight = hardwareMap.dcMotor.get("REAR_RIGHT_MOTOR");

    // Reverse the right side motors
    // Reverse left motors if you are using NeveRests
    motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    while (opModeIsActive()) {

      buttonTimerCurrentLoopTime = buttonTimer.milliseconds();

      double g2LY = gamepad2.left_stick_y;
      double g2LX = gamepad2.left_stick_x;

      if (g2LY > 0
          && (buttonTimerCurrentLoopTime - buttonTimerPrevLoopTimeUD)
              > tapeMeasureUDServoTickMS / g2LY) { // tips tape up
        tapeMeasureUDServoPosition += tapeMeasureUDServoTick;
        udServo.setPosition(tapeMeasureUDServoPosition);
        buttonTimerPrevLoopTimeUD = buttonTimerCurrentLoopTime;
      }

      if (g2LY < 0
          && (buttonTimerCurrentLoopTime - buttonTimerPrevLoopTimeUD)
              > tapeMeasureUDServoTickMS / -g2LY) { // tips tape down
        tapeMeasureUDServoPosition -= tapeMeasureUDServoTick;
        udServo.setPosition(tapeMeasureUDServoPosition);
        buttonTimerPrevLoopTimeUD = buttonTimerCurrentLoopTime;
      }

      if (g2LX > 0
          && (buttonTimerCurrentLoopTime - buttonTimerPrevLoopTimeRL)
              > tapeMeasureRLServoTickMS / g2LX) { // turn right
        tapeMeasureRLServoPosition -= tapeMeasureRLServoTick;
        rlServo.setPosition(tapeMeasureRLServoPosition);
        buttonTimerPrevLoopTimeRL = buttonTimerCurrentLoopTime;
      }

      if (g2LX < 0
          && (buttonTimerCurrentLoopTime - buttonTimerPrevLoopTimeRL)
              > tapeMeasureRLServoTickMS / -g2LX) { // turn left
        tapeMeasureRLServoPosition += tapeMeasureRLServoTick;
        rlServo.setPosition(tapeMeasureRLServoPosition);
        buttonTimerPrevLoopTimeRL = buttonTimerCurrentLoopTime;
      }

      if (gamepad2.right_trigger > 0) { // extends the tape measure
        extensionServo.setPower(-1);
      } else if (gamepad2.left_trigger > 0) { // retracts the tape measure
        extensionServo.setPower(1);
      } else extensionServo.setPower(0); // stopped

      double g1Y = -gamepad1.left_stick_y;
      double g1X = gamepad1.left_stick_x;
      double g1RX = gamepad1.right_stick_x;

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio, but only when
      // at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(g1Y) + Math.abs(g1X) + Math.abs(g1RX), 1);
      double frontLeftPower = (g1Y + g1X + g1RX) / denominator;
      double backLeftPower = (g1Y - g1X + g1RX) / denominator;
      double frontRightPower = (g1Y - g1X - g1RX) / denominator;
      double backRightPower = (g1Y + g1X - g1RX) / denominator;

      motorFrontLeft.setPower(frontLeftPower);
      motorBackLeft.setPower(backLeftPower);
      motorFrontRight.setPower(frontRightPower);
      motorBackRight.setPower(backRightPower);

      //telemetry.addData("RL Servo: ", tapeMeasureRLServoPosition);
      //telemetry.addData("UD Servo: ", tapeMeasureUDServoPosition);
      telemetry.addData("LeftStickX", gamepad1.left_stick_x);
      telemetry.addData("LeftStickY", gamepad1.left_stick_y);
      telemetry.addData("RightStickX", gamepad1.right_stick_x);
      telemetry.addData("RightStickY", gamepad1.right_stick_y);
      telemetry.update();
    }
  }
}
