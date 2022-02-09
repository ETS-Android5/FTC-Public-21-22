package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        double buttonTimerPrevLoopTime = 0;
        double buttonTimerSinceLastButton;

        // RL is Right/Left servo.
        double tapeMeasureRLServoPosition = .5; // This is init position and tracking of current position
        final double tapeMeasureRLServoTickMS = 200; // This is milliseconds between allowed movements of each tick
        final double tapeMeasureRLServoTick = .005; // This is amount of each tick

        // UD is Up/Down servo.
        double tapeMeasureUDServoPosition = .5; // This is init position and tracking of current position
        final double tapeMeasureUDServoTickMS = 200; // This is milliseconds between allowed movements of each tick
        final double tapeMeasureUDServoTick = .005; // This is amount of each tick

        // init RL servo to tapeMeasureRLServoPosition

        // init UD servo to tapeMeasureUDServoPosition


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo yawServo = hardwareMap.servo.get("YAW_SERVO");
        Servo pitchServo = hardwareMap.servo.get("PITCH_SERVO");


        waitForStart();

        while (opModeIsActive()) {

            buttonTimerCurrentLoopTime = buttonTimer.milliseconds();
            buttonTimerSinceLastButton = buttonTimerCurrentLoopTime - buttonTimerPrevLoopTime;

            if (gamepad1.a && buttonTimerSinceLastButton > tapeMeasureUDServoTickMS) {    // gamepad A is up
                tapeMeasureUDServoPosition += tapeMeasureUDServoTick;
                // set UD server to tapeMeasureUDServoPosition
                buttonTimerPrevLoopTime = buttonTimerCurrentLoopTime;
            }

            if (gamepad1.y && buttonTimerSinceLastButton > tapeMeasureUDServoTickMS) {    // gamepad A is up
                tapeMeasureUDServoPosition -= tapeMeasureUDServoTick;
                // set UD server to tapeMeasureUDServoPosition
                buttonTimerPrevLoopTime = buttonTimerCurrentLoopTime;
            }

            if (gamepad1.dpad_right && buttonTimerSinceLastButton > tapeMeasureRLServoTickMS) {    // gamepad dpad_right is right
                tapeMeasureRLServoPosition += tapeMeasureRLServoTick;
                // set RL server to tapeMeasureRLServoPosition
                buttonTimerPrevLoopTime = buttonTimerCurrentLoopTime;
            }

            if (gamepad1.dpad_left && buttonTimerSinceLastButton > tapeMeasureRLServoTickMS) {    // gamepad dpad_left is left
                tapeMeasureRLServoPosition -= tapeMeasureRLServoTick;
                // set RL server to tapeMeasureRLServoPosition
                buttonTimerPrevLoopTime = buttonTimerCurrentLoopTime;
            }

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            yawServo.setPosition(tapeMeasureRLServoPosition);
            pitchServo.setPosition(tapeMeasureUDServoPosition);
        }
    }
}
