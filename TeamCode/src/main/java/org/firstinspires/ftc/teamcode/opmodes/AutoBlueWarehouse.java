package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.SliderLiftBot;
import org.opencv.core.Mat;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "CB_AUTO_AutoBlue")
public class AutoBlueWarehouse extends EnhancedAutonomous {

  private final SliderLiftBot robot;

  public AutoBlueWarehouse() {
    super(new SliderLiftBot());
    this.robot = (SliderLiftBot) robotObject;
  }

  @Override
  public void onInitPressed() {
    OpenCVWrapper.load();
  }

  @Override
  public void onStartPressed() {
    AtomicReference<Mat> img = new AtomicReference<>();
    AtomicReference<TeamMarkerPosition> teamMarkerPosition =
        new AtomicReference<>(TeamMarkerPosition.RIGHT);
    Thread worker =
        new Thread(
            () -> {
              robot.webcam.start();
              img.set(robot.webcam.grabFrame());
            });
    worker.start();
    robot.drivetrain.setPowerCurve(PowerCurves.generatePowerCurve(0.25, 2.33));
    processChanges();
    try {
      worker.join();
      worker =
          new Thread(
              () ->
                  teamMarkerPosition.set(
                      new TeamMarkerPositionDetector()
                          .calculateTeamMarkerPosition(
                              img.get(), CameraPosition.REAR_LEFT_FACING_DOWN)));
      worker.start();
    } catch (InterruptedException e) {
      StringWriter sw = new StringWriter();
      PrintWriter pw = new PrintWriter(sw);
      e.printStackTrace(pw);
    }
    robot.drivetrain.setFrontLeftPower(0.3);
    robot.drivetrain.setFrontRightPower(-0.3);
    robot.drivetrain.setRearLeftPower(0.3);
    robot.drivetrain.setRearRightPower(-0.3);
    processChanges();
    sleep(375);
    robot.drivetrain.setAllPower(0);
    processChanges();
    try {
      worker.join();
    } catch (InterruptedException e) {
      StringWriter sw = new StringWriter();
      PrintWriter pw = new PrintWriter(sw);
      e.printStackTrace(pw);
    }
    switch (teamMarkerPosition.get()) {
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
    int distanceToShippingHubTime =
        teamMarkerPosition.get() == TeamMarkerPosition.RIGHT ? 1900 : 2000;
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

    robot.drivetrain.setFrontLeftPower(0.25);
    robot.drivetrain.setFrontRightPower(-0.25);
    robot.drivetrain.setRearLeftPower(0.25);
    robot.drivetrain.setRearRightPower(-0.25);
    processChanges();
    sleep(500);
    robot.drivetrain.setFrontLeftPower(-0.5);
    robot.drivetrain.setFrontRightPower(0.5);
    robot.drivetrain.setRearLeftPower(0.5);
    robot.drivetrain.setRearRightPower(-0.5);
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
