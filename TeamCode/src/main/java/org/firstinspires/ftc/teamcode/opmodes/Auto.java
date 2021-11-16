package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.IntegerRes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.CallbackData;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InitializedFilterPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ResetDcMotorPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.RunToPositionPipe;
import org.firstinspires.ftc.teamcode.core.opmodes.Constants;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="Uhoh")
public class Auto extends EnhancedAutonomous {

    private final MecanumBot robot = new MecanumBot();

    public Auto() {
        super(new HardwarePipeline(
                Constants.PIPELINE_BASE_NAME,
                new InitializedFilterPipe(
                        "FilterElement",
                        new ResetDcMotorPipe(
                                "MotorReset",
                                new MotorTrackerPipe(
                                        "MotorTracker",
                                        new RunToPositionPipe(
                                                "RunToPosition",
                                                new ExitPipe("Exit")
                                        )
                                )
                        )
                )
        ));
        super.initialize(robot);
    }

    @Override
    public void onInitPressed() {}

    @Override
    public void onStartPressed() {
        int ticksPerInch = 15;
        robot.drivetrain.setRunMode(RunMode.RUN_TO_POSITION);
        robot.drivetrain.setPowerCurve(PowerCurves.RUN_TO_POSITION_HALF_POWER);
        AtomicBoolean reachedFrontLeftTarget = new AtomicBoolean(false);
        AtomicBoolean reachedFrontRightTarget = new AtomicBoolean(false);
        AtomicBoolean reachedRearLeftTarget = new AtomicBoolean(false);
        AtomicBoolean reachedRearRightTarget = new AtomicBoolean(false);
        int[] targetPositions = {15, 105, 285, 300, 390, 570, 585, 675, 855};
        for (int target : targetPositions) {
            robot.drivetrain.setAllTarget(target);
            MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                    robot.drivetrain.getFrontLeftName(),
                    target,
                    5,
                    () -> reachedFrontLeftTarget.set(true)
            ));
            MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                    robot.drivetrain.getFrontRightName(),
                    target,
                    5,
                    () -> reachedFrontRightTarget.set(true)
            ));
            MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                    robot.drivetrain.getRearLeftName(),
                    target,
                    5,
                    () -> reachedRearLeftTarget.set(true)
            ));
            MotorTrackerPipe.getInstance().setCallbackForMotorPosition(new CallbackData(
                    robot.drivetrain.getRearRightName(),
                    target,
                    5,
                    () -> reachedRearRightTarget.set(true)
            ));
            while (opModeIsActive()
                    && !reachedFrontLeftTarget.get()
            && !reachedFrontRightTarget.get()
            && !reachedRearLeftTarget.get()
            && !reachedRearRightTarget.get()) {
                hardwarePipeline.process(initializedHardware, robot);
            }
            reachedFrontLeftTarget.set(false);
            reachedFrontRightTarget.set(false);
            reachedRearLeftTarget.set(false);
            reachedRearRightTarget.set(false);
            sleep(4000);
        }
    }
}
