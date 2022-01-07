package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;

@Autonomous(name = "CB_AUTO_DistanceTest")
@SuppressWarnings("unused")
public class DistanceTest extends EnhancedAutonomous {

  private final TurretBot robot;

  public DistanceTest() {
    super(new TurretBot(Alliance.RED));
    this.robot = (TurretBot) super.robotObject;
  }

  @Override
  public void onInitPressed() {
    OpenCVWrapper.load();
  }

  @Override
  public void onStartPressed() {
    robot.frontRange.startSampling();
    robot.gyro.startSampling();
    processChanges();
    processChanges();
    processChanges();
    processChanges();
    processChanges();
    robot.turnToHeading(0, 1, super::opModeIsActive, super::processChanges);
    robot.runAtHeadingUntilCondition(
        0,
        Direction.FORWARD,
        () ->
            Math.abs(
                    InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName())
                        - 400)
                <= 25,
        () -> 0.25,
        super::opModeIsActive,
        super::processChanges);
  }

  @Override
  public void onStop() {
    robot.webcam.deinit();
  }
}
