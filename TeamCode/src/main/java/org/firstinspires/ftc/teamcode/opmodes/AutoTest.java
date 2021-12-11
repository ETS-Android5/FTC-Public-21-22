package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.core.fn.PowerCurves.RUN_TO_POSITION_QUARTER_POWER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;

@Autonomous(name = "Oh boy")
public class AutoTest extends EnhancedAutonomous {

  private final MecanumBot robot;

  public AutoTest() {
    super(new MecanumBot());
    this.robot = (MecanumBot) robotObject;
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void onStartPressed() {
    robot.gyro.initialize();
    robot.gyro.startSampling();
    robot.rearDistance.startSampling();
    robot.drivetrain.setRunMode(RunMode.RUN_USING_ENCODER);
    processChanges();
    if (!opModeIsActive()) {
      return;
    }
    double start =
        InterpolatablePipe.getInstance().currentDataPointOf(robot.rearDistance.getName());
    robot.runAtHeadingUntilCondition(
        0,
        1,
        Direction.FORWARD,
        () -> {
          double dist =
              InterpolatablePipe.getInstance().currentDataPointOf(robot.rearDistance.getName());
          return dist <= 305 + 5 && dist >= 305 - 5;
        },
        () -> {
          double dist =
              InterpolatablePipe.getInstance().currentDataPointOf(robot.rearDistance.getName());
          return RUN_TO_POSITION_QUARTER_POWER.apply(
              (int) Math.round(dist), (int) Math.round(start), 305);
        },
        super::opModeIsActive,
        super::processChanges);
  }

  @Override
  public void onStop() {}
}
