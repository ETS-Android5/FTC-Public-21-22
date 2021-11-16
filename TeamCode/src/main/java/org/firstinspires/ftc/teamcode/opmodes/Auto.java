package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.HardwarePipeline;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InitializedFilterPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ResetDcMotorPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.RunToPositionPipe;
import org.firstinspires.ftc.teamcode.core.opmodes.Constants;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;

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
        robot.drivetrain.setRunMode(RunMode.RUN_TO_POSITION);
        robot.drivetrain.setPowerCurve(PowerCurves.RUN_TO_POSITION_QUARTER_POWER);
        hardwarePipeline.process(initializedHardware, robot);
        int[] targetPositions = {15, 105, 285, 300, 390, 570, 585, 675, 855};
        for (int target : targetPositions) {
            robot.drivetrain.autoRunToPosition(target,
                    6,
                    super::opModeIsActive,
                    () -> hardwarePipeline.process(initializedHardware, robot));
            sleep(4000);
        }
    }
}
