package org.firstinspires.ftc.teamcode.opmodes.auto.blue.carousel;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.cv.ViewPortDescription;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "CB_AUTO_BlueCuracao")
@SuppressWarnings("unused")
public class BlueCuracao extends EnhancedAutonomous {
  private final TurretBot robot;

  public BlueCuracao() {
    super(new TurretBot(Alliance.RED, TurretBot.AUTO_FIRST_JOINT_OFFSET, true));
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
    wait(delay);
    robot.lift.setArmOnePosition(570);
    robot.turretAdjustment.addAndGet(-10 * Turret.DEGREES_TO_TICKS);
    try {
      worker.join();
      worker =
          new Thread(
              () ->
                  teamMarkerPosition.set(
                      new TeamMarkerPositionDetector()
                          .calculateTeamMarkerPosition(
                              img.get(),
                              CameraPosition.REAR_LOW_AND_CENTERED,
                              ViewPortDescription.LEFT_TWO_IN_VIEW)));
      worker.start();
    } catch (InterruptedException e) {
      Log.e("TURRETBOT", "ERROR", e);
    }
    wait(1000);
    try {
      worker.join();
    } catch (InterruptedException e) {
      Log.e("TURRETBOT", "ERROR", e);
    }
    switch (teamMarkerPosition.get()) {
      case LEFT:
        Log.d("TURRETBOT", "SAW LEFT");
        robot.goToPosition(TurretBotPosition.AUTO_REACH_BOTTOM, false);
        break;
      case CENTER:
        Log.d("TURRETBOT", "SAW CENTER");
        robot.goToPosition(TurretBotPosition.AUTO_REACH_MIDDLE, false);
        break;
      case RIGHT:
        Log.d("TURRETBOT", "SAW RIGHT");
        robot.goToPosition(TurretBotPosition.AUTO_REACH_TOP, false);
        break;
    }
    theRestOfCuracao(teamMarkerPosition.get() != TeamMarkerPosition.RIGHT);
  }

  private void theRestOfCuracao(boolean shouldTurnTurretLeft) {
    wait(2000);
    robot.driveForward(.5, 0.5, -19, 20, super::opModeIsActive, super::processChanges);
    wait(robot.dropFreight());
    robot.driveForward(.5, 0.5, 6, 20, super::opModeIsActive, super::processChanges);
    if (shouldTurnTurretLeft) {
      robot.turret.turnToPosition(Turret.TICKS_RIGHT);
    } else {
      robot.turret.turnToPosition(robot.turretAdjustment.get());
    }
    wait(1000);
    robot.lift.setArmOnePosition(570);
    wait(1000);
    robot.lift.setArmTwoPosition(0.86);
    wait(1000);
    robot.turret.turnToPosition(robot.turretAdjustment.get());
    wait(1000);
    robot.driveForward(.5, 0.5, 9, 20, super::opModeIsActive, super::processChanges);
    robot.turnToHeading(-93, 0.5, super::opModeIsActive, super::processChanges);
    wait(300);
    robot.turnToHeading(-93, 0.5, super::opModeIsActive, super::processChanges);
    retreat();
  }

  private void retreat() {
    robot.driveForward(.5, 1.5, -32, 20, super::opModeIsActive, super::processChanges);
    wait(500);
    robot.turnToHeading(-95, 0.5, super::opModeIsActive, super::processChanges);
    robot.carouselSpinner.spinBackward();
    wait(4000);
    robot.drivetrain.driveBySticks(0.6, 0, 0);
    wait(1100);
    robot.driveForward(.3, 1.5, -2, 20, super::opModeIsActive, super::processChanges);
    robot.intake.beginIntaking();
    robot.lift.setArmTwoPosition(TurretBotPosition.INTAKE_POSITION.secondJointTarget());
    wait(250);
    robot.intake.stop();
    robot.goToPosition(TurretBotPosition.INTAKE_POSITION, false);
    wait(6000);
  }

  @Override
  public void onStop() {}
}
