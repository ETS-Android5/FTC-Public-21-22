package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;

@Autonomous(name = "Uhoh")
public class Auto extends EnhancedAutonomous {

  private final MecanumBot robot;

  public Auto() {
    super(new MecanumBot());
    this.robot = (MecanumBot) robotObject;
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void onStartPressed() {
    robot.drivetrain.setFrontLeftDirection(robot.drivetrain.getFrontLeftDirection().opposite());
    robot.drivetrain.setFrontRightDirection(robot.drivetrain.getFrontRightDirection().opposite());
    robot.drivetrain.setRunMode(RunMode.RUN_TO_POSITION);
    robot.drivetrain.setAllTarget(0);
    robot.drivetrain.setPowerCurve(
        (Integer a, Integer b, Integer c) ->
            (double) Math.round(PowerCurves.RUN_TO_POSITION_QUARTER_POWER.apply(a, b, c) * 100)
                / 100);
    processChanges();
    int[] targetPositions = {15, 105, 285, 300, 390, 570, 585, 675, 855};
    for (int target : targetPositions) {
      robot.drivetrain.autoRunToPosition(target, 10, super::opModeIsActive, super::processChanges);
      sleep(4000);
    }
  }
}
