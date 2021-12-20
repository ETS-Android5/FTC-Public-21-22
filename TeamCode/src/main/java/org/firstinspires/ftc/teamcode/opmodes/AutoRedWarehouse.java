package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.MecanumBot;
import org.opencv.core.Mat;

import java.io.PrintWriter;
import java.io.StringWriter;

@Autonomous(name = "CB_AUTO_AutoRed")
public class AutoRedWarehouse extends EnhancedAutonomous {

  private final MecanumBot robot;

  public AutoRedWarehouse() {
    super(new MecanumBot());
    this.robot = (MecanumBot) robotObject;
  }

  @Override
  public void onInitPressed() {
    OpenCVWrapper.load();
  }

  @Override
  public void onStartPressed() {
    Thread worker =
        new Thread(
            () -> {
              robot.webcam.init();
              robot.webcam.start();
            });
    worker.start();
    robot.drivetrain.setPowerCurve(PowerCurves.generatePowerCurve(0.25, 2.33));
    processChanges();
    try {
      worker.join();
    } catch (InterruptedException e) {
      StringWriter sw = new StringWriter();
      PrintWriter pw = new PrintWriter(sw);
      e.printStackTrace(pw);
    }
    Mat frame = robot.webcam.grabFrame();
    TeamMarkerPosition teamMarkerPosition =
        new TeamMarkerPositionDetector()
            .calculateTeamMarkerPosition(frame, CameraPosition.REAR_LEFT_FACING_DOWN);

    robot.drivetrain.setFrontLeftPower(-0.3);
    robot.drivetrain.setFrontRightPower(0.3);
    robot.drivetrain.setRearLeftPower(-0.3);
    robot.drivetrain.setRearRightPower(0.3);
    processChanges();
    sleep(330);
    robot.drivetrain.setAllPower(0);
    processChanges();
    // frontRightTarget -= 250;
    // rearRightTarget -= 500;
    // robot.drivetrain.autoRunToPosition(frontLeftTarget, frontRightTarget, rearLeftTarget,
    // rearRightTarget, 5, super::opModeIsActive, super::processChanges);
    // sleep(2000);
    // front backwards
    // rear forwards
    // strafes left
    // front forwards
    // rear backwards
    // strafes right
    switch (teamMarkerPosition) {
      case LEFT:
        robot.fourHeightLift.goToHeight1();
        break;
      case CENTER:
        robot.fourHeightLift.goToHeight2();
        break;
      case RIGHT:
        robot.fourHeightLift.goToHeight3();
        break;
    }
    long start = System.nanoTime();
    while (opModeIsActive() && ((System.nanoTime() - start) / 1000000 < 2000)) {
      processChanges();
    }
    int distanceToShippingHubTime = teamMarkerPosition == TeamMarkerPosition.RIGHT ? 1500 : 1800;
    robot.drivetrain.setAllPower(-.25);
    processChanges();
    sleep(distanceToShippingHubTime);
    robot.drivetrain.setAllPower(0);
    processChanges();
    robot.outtakeBucket.dump();
    processChanges();
    sleep(750);
    robot.outtakeBucket.carry();
    processChanges();
    sleep(750);

    robot.drivetrain.setAllPower(.25);
    processChanges();
    sleep(distanceToShippingHubTime + 1000);
    robot.drivetrain.setAllPower(0);
    processChanges();

    robot.fourHeightLift.goToHeight0();
    start = System.nanoTime();
    while (opModeIsActive() && ((System.nanoTime() - start) / 1000000 < 2000)) {
      processChanges();
    }

    robot.drivetrain.setFrontLeftPower(-0.25);
    robot.drivetrain.setFrontRightPower(0.25);
    robot.drivetrain.setRearLeftPower(-0.25);
    robot.drivetrain.setRearRightPower(0.25);
    processChanges();
    sleep(500);
    robot.drivetrain.setFrontLeftPower(0.5);
    robot.drivetrain.setFrontRightPower(-0.5);
    robot.drivetrain.setRearLeftPower(-0.5);
    robot.drivetrain.setRearRightPower(0.5);
    processChanges();
    sleep(800);
    robot.drivetrain.setAllPower(0);
    robot.intake.lower();
    processChanges();
    sleep(1000);

    robot.drivetrain.setAllPower(.6);

    processChanges();
    sleep(1000);
    robot.drivetrain.setAllPower(0);
    processChanges();
    sleep(1000);

    robot.fourHeightLift.goToHeight0();
    start = System.nanoTime();
    while (opModeIsActive() && ((System.nanoTime() - start) / 1000000 < 2000)) {
      processChanges();
    }
  }

  @Override
  public void onStop() {
    robot.webcam.deinit();
  }
}
