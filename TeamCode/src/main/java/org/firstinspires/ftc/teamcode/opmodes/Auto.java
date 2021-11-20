package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.ITeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

@Autonomous(name = "Uhoh")
public class Auto extends EnhancedAutonomous {

  private final MecanumBot robot;

  public Auto() {
    super(new MecanumBot());
    this.robot = (MecanumBot) robotObject;
  }

  @Override
  public void onInitPressed() {
    OpenCVLoader.initDebug();
  }

  @Override
  public void onStartPressed() {
    robot.webcam.init();
    robot.drivetrain.setFrontLeftDirection(robot.drivetrain.getFrontLeftDirection().opposite());
    robot.drivetrain.setFrontRightDirection(robot.drivetrain.getFrontRightDirection().opposite());
    robot.drivetrain.setRunMode(RunMode.RUN_TO_POSITION);
    robot.drivetrain.setAllTarget(0);
    robot.drivetrain.setPowerCurve(
        (Integer a, Integer b, Integer c) ->
            (double) Math.round(PowerCurves.RUN_TO_POSITION_QUARTER_POWER.apply(a, b, c) * 100)
                / 100);
    processChanges();
    ITeamMarkerPositionDetector markerPositionDetector = new TeamMarkerPositionDetector();
    Mat frame = robot.webcam.grabFrame();
    robot.webcam.deinit();
    TeamMarkerPosition teamMarkerPosition = markerPositionDetector.calculateTeamMarkerPosition(frame);
    robot.drivetrain.autoRunToPosition(-1000, 5, super::opModeIsActive, super::processChanges);
    robot.fourHeightLift.goToHeight3();

    /*
    int[] targetPositions = {1000, -2000};
    for (int target : targetPositions) {
      robot.drivetrain.autoRunToPosition(target, 5, super::opModeIsActive, super::processChanges);
      sleep(4000);
    }
    */
  }
}
