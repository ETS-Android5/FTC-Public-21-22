package org.firstinspires.ftc.teamcode.opmodes.auto.blue.warehouse;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "CB_AUTO_BlueWBarrier")
@SuppressWarnings("unused")
public class AutoBlueWarehouseEndBarrier extends EnhancedAutonomous {
  private static final double FRONT_DISTANCE_FROM_WALL_0 = 448; // Ticks
  private static final double FRONT_DISTANCE_FROM_WALL_BOTTOM = 350; // Ticks
  private static final double FRONT_DISTANCE_FROM_WALL_MIDDLE = 350; // Ticks
  private static final double FRONT_DISTANCE_FROM_WALL_TOP = 470; // Ticks
  private static final double LEFT_DISTANCE_FROM_WALL_0 = 18.1; // Inches
  private final TurretBot robot;

  public AutoBlueWarehouseEndBarrier() {
    super(new TurretBot(Alliance.BLUE, TurretBot.AUTO_FIRST_JOINT_OFFSET, true));
    this.robot = (TurretBot) super.robotObject;
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
              new Thread(robot.webcam::deinit).start();
            });
    worker.start();
    robot.gyro.startSampling();
    robot.lift.setArmTwoPosition(0.86);
    int delay = robot.grabFreight();
    processChanges();
    sleep(delay);
    robot.lift.setArmOnePosition(550);
    processChanges();
    try {
      worker.join();
      worker =
          new Thread(
              () ->
                  teamMarkerPosition.set(
                      new TeamMarkerPositionDetector()
                          .calculateTeamMarkerPosition(
                              img.get(), CameraPosition.REAR_LOW_AND_CENTERED)));
      worker.start();
    } catch (InterruptedException e) {
      Log.e("TURRETBOT", "ERROR", e);
    }
    robot.drivetrain.setAllPower(-.25);
    double startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive()
        && startAvg - robot.drivetrain.avgEncoderValue() <= FRONT_DISTANCE_FROM_WALL_0) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    robot.drivetrain.driveBySticks(-1, 0, 0);
    long start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < 250) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    robot.turnToHeading(-43, 0.5, super::opModeIsActive, super::processChanges);
    try {
      worker.join();
    } catch (InterruptedException e) {
      Log.e("TURRETBOT", "ERROR", e);
    }
    switch (teamMarkerPosition.get()) {
      case LEFT:
        Log.d("TURRETBOT", "SAW LEFT");
        handleBottomPosition();
        break;
      case CENTER:
        Log.d("TURRETBOT", "SAW CENTER");
        handleMiddlePosition();
        break;
      case RIGHT:
        Log.d("TURRETBOT", "SAW RIGHT");
        handleTopPosition();
        break;
    }
  }

  private void handleBottomPosition() {
    robot.setAlliance(Alliance.RED);
    robot.goToPosition(TurretBotPosition.ALLIANCE_BOTTOM_POSITION, false);
    long start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
      processChanges();
    }
    robot.setAlliance(Alliance.BLUE);
    robot.drivetrain.setAllPower(-.25);
    double startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive()
        && startAvg - robot.drivetrain.avgEncoderValue() <= FRONT_DISTANCE_FROM_WALL_BOTTOM) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    int delay = robot.dropFreight();
    start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < delay) {
      processChanges();
    }
    retreat(FRONT_DISTANCE_FROM_WALL_BOTTOM);
  }

  private void handleMiddlePosition() {
    robot.setAlliance(Alliance.RED);
    robot.goToPosition(TurretBotPosition.ALLIANCE_MIDDLE_POSITION, false);
    long start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
      processChanges();
    }
    robot.setAlliance(Alliance.BLUE);
    robot.drivetrain.setAllPower(-.25);
    double startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive()
        && startAvg - robot.drivetrain.avgEncoderValue() <= FRONT_DISTANCE_FROM_WALL_MIDDLE) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    int delay = robot.dropFreight();
    start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < delay) {
      processChanges();
    }
    retreat(FRONT_DISTANCE_FROM_WALL_MIDDLE);
  }

  private void handleTopPosition() {
    robot.setAlliance(Alliance.RED);
    robot.goToPosition(TurretBotPosition.ALLIANCE_TOP_POSITION, false);
    long start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
      processChanges();
    }
    robot.setAlliance(Alliance.BLUE);
    robot.drivetrain.setAllPower(-.25);
    double startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive()
        && startAvg - robot.drivetrain.avgEncoderValue() <= FRONT_DISTANCE_FROM_WALL_TOP) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    int delay = robot.dropFreight();
    start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < delay) {
      processChanges();
    }
    retreat(FRONT_DISTANCE_FROM_WALL_TOP);
  }

  private void retreat(double preMoveFrontDistance) {
    robot.drivetrain.setAllPower(0.25);
    double startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive()
        && startAvg + preMoveFrontDistance <= robot.drivetrain.avgEncoderValue()) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    robot.turret.turnToPosition(Turret.TICKS_LEFT);
    long turretStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - turretStart < 2000) {
      processChanges();
    }
    robot.grabTeamMarker();
    robot.goToPosition(TurretBotPosition.INTAKE_POSITION, false);
    robot.turnToHeading(-90, 0.5, super::opModeIsActive, super::processChanges);
    long intakeStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - intakeStart < 1000) {
      processChanges();
    }
    robot.drivetrain.driveBySticks(-.8, 0, 0);
    long start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < 800) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0.5);
    start = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start < 1750) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    while (opModeIsActive()) {
      processChanges();
    }
  }

  @Override
  public void onStop() {}
}
