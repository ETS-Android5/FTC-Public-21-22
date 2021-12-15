package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MotorPowerTest")
public class Test3 extends OpMode {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor rl;
    private DcMotor rr;

    boolean flRunning = false;
    boolean frRunning = false;
    boolean rlRunning = false;
    boolean rrRunning = false;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("FRONT_LEFT_MOTOR");
        fr = hardwareMap.dcMotor.get("FRONT_RIGHT_MOTOR");
        rl = hardwareMap.dcMotor.get("REAR_LEFT_MOTOR");
        rr = hardwareMap.dcMotor.get("REAR_RIGHT_MOTOR");

        reset(fl);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        reset(fr);
        reset(rl);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        reset(rr);
    }

    private void reset(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            flRunning = !flRunning;
        }
        if (gamepad1.b) {
            frRunning = !frRunning;
        }
        if (gamepad1.x) {
            rlRunning = !rlRunning;
        }
        if (gamepad1.y) {
            rrRunning = !rrRunning;
        }

        fl.setPower(flRunning ? 1 : 0);
        fr.setPower(frRunning ? 1 : 0);
        rl.setPower(rlRunning ? 1 : 0);
        rr.setPower(rrRunning ? 1 : 0);

        telemetry.addData("FL", fl.getCurrentPosition());
        telemetry.addData("FR", fr.getCurrentPosition());
        telemetry.addData("RL", rl.getCurrentPosition());
        telemetry.addData("RR", rr.getCurrentPosition());
    }
}
