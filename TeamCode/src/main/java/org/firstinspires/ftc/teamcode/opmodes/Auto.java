package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        long start = System.currentTimeMillis();
        robot.drivetrain.setAllPower(1);
        hardwarePipeline.process(initializedHardware, robot);
        while (System.currentTimeMillis() - start < 2000 && opModeIsActive()) {

        }
        robot.drivetrain.setAllPower(0);
    }
}
