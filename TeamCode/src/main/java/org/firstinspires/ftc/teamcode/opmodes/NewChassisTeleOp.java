package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InitializedFilterPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ResetDcMotorPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.RunToPositionPipe;
import org.firstinspires.ftc.teamcode.core.opmodes.Constants;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name="NewChassis")
public class NewChassisTeleOp extends EnhancedTeleOp {
    private static double THIRD_MANIPULATION(double in) {
        return Math.pow(in, 3);
    }

    private final NewChassis robot;

    private final AtomicBoolean halfSpeed = new AtomicBoolean(false);

    public NewChassisTeleOp() {
        super(new NewChassis());
        this.robot = (NewChassis) super.robotObject;
    }

    @Override
    public void onInitPressed() {

    }

    @Override
    public void initLoop() {

    }

    @Override
    public void onStartPressed() {
        controller1.setManipulation(NewChassisTeleOp::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);
        controller1.registerOnPressedCallback(() -> halfSpeed.set(!halfSpeed.get()), true, BooleanSurface.X);

    }

    @Override
    public void onLoop() {
        double turnValue = controller1.rightStickX();
        double speed = halfSpeed.get() ? 0.5 : 1;
        robot.drivetrain.driveBySticks(
                controller1.leftStickX() * speed,
                controller1.leftStickY() * speed,
                turnValue * speed
        );
    }

    @Override
    public void onStop() {

    }
}
