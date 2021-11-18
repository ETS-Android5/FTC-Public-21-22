package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous()
public class Ugh extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    private void setRunMode(DcMotor.RunMode r) {
        frontLeftMotor.setMode(r);
        frontRightMotor.setMode(r);
        rearLeftMotor.setMode(r);
        rearRightMotor.setMode(r);
    }

    private void setAllTargetPosition(int ticks) {
        frontLeftMotor.setTargetPosition(ticks);
        frontRightMotor.setTargetPosition(ticks);
        rearLeftMotor.setTargetPosition(ticks);
        rearRightMotor.setTargetPosition(ticks);
    }

    private void setAllPower(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("FRONT_LEFT_MOTOR");
        frontRightMotor = hardwareMap.dcMotor.get("FRONT_RIGHT_MOTOR");
        rearLeftMotor = hardwareMap.dcMotor.get("REAR_LEFT_MOTOR");
        rearRightMotor = hardwareMap.dcMotor.get("REAR_RIGHT_MOTOR");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        for (int tick : new int[] {15, 105, 285, 300, 390, 570, 585, 675, 855}) {
            setAllTargetPosition(tick);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            setAllPower(.25);
            boolean targetReached = false;
            while (opModeIsActive() && !targetReached) {
                int flCurrent = frontLeftMotor.getCurrentPosition();
                int frCurrent = frontRightMotor.getCurrentPosition();
                int rlCurrent = rearLeftMotor.getCurrentPosition();
                int rrCurrent = rearRightMotor.getCurrentPosition();
                boolean flTargetReached = flCurrent >= (tick - 6) && flCurrent <= (tick + 6);
                boolean frTargetReached = frCurrent >= (tick - 6) && frCurrent <= (tick + 6);
                boolean rlTargetReached = rlCurrent >= (tick - 6) && rlCurrent <= (tick + 6);
                boolean rrTargetReached = rrCurrent >= (tick - 6) && rrCurrent <= (tick + 6);
                targetReached = flTargetReached && frTargetReached && rlTargetReached && rrTargetReached;
            }
            sleep(1000);
        }
    }
}
