package org.firstinspires.ftc.teamcode.opmodes.auto.blue.warehouse;

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

@Autonomous(name = "CB_AUTO_Blue_Warehouse")
@SuppressWarnings("unused")
public class AutoBlueWarehouse extends EnhancedAutonomous {
  private final TurretBot robot;

  public AutoBlueWarehouse() {
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
        new AtomicReference<>(TeamMarkerPosition.LEFT);
    Thread worker =
        new Thread(
            () -> {
              robot.webcam.start();
              img.set(robot.webcam.grabFrame());
              new Thread(robot.webcam::deinit).start();
            });
    worker.start();
    robot.gyro.startSampling();
    robot.lift.setArmTwoPosition(0.885);
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
                              ViewPortDescription.RIGHT_TWO_IN_VIEW)));
      worker.start();
    } catch (InterruptedException e) {
      Log.e("TURRETBOT", "ERROR", e);
    }
    wait(1500);
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
    robot.goToPosition(TurretBotPosition.AUTO_REACH_BOTTOM, false);
    wait(2000);
    robot.driveForward(.5, 0.5, -19, 20, super::opModeIsActive, super::processChanges);
    wait(robot.dropFreight());
    robot.driveForward(.5, 0.5, 12, 20, super::opModeIsActive, super::processChanges);
    robot.turret.turnToPosition(Turret.TICKS_LEFT);
    wait(1000);
    robot.lift.setArmOnePosition(570);
    wait(1000);
    robot.lift.setArmTwoPosition(0.885);
    wait(1000);
    robot.turret.turnToPosition(robot.turretAdjustment.get());
    wait(1000);
    robot.turnToHeading(90, 0.5, super::opModeIsActive, super::processChanges);
    retreat();
  }

  private void handleMiddlePosition() {
    robot.goToPosition(TurretBotPosition.AUTO_REACH_MIDDLE, false);
    wait(1000);
    robot.driveForward(.5, 0.5, -19, 20, super::opModeIsActive, super::processChanges);
    wait(robot.dropFreight());
    robot.driveForward(.5, 0.5, 12, 20, super::opModeIsActive, super::processChanges);
    robot.turret.turnToPosition(Turret.TICKS_LEFT);
    wait(1000);
    robot.lift.setArmOnePosition(570);
    wait(1000);
    robot.lift.setArmTwoPosition(0.885);
    wait(1000);
    robot.turret.turnToPosition(robot.turretAdjustment.get());
    wait(1000);
    robot.turnToHeading(90, 0.5, super::opModeIsActive, super::processChanges);
    retreat();
  }

  private void handleTopPosition() {
    robot.goToPosition(TurretBotPosition.AUTO_REACH_TOP, false);
    wait(1000);
    robot.driveForward(.5, 0.5, -19, 20, super::opModeIsActive, super::processChanges);
    wait(robot.dropFreight());
    robot.driveForward(.5, 0.5, 12, 20, super::opModeIsActive, super::processChanges);
    robot.turret.turnToPosition(robot.turretAdjustment.get());
    wait(1000);
    robot.lift.setArmOnePosition(570);
    wait(1000);
    robot.lift.setArmTwoPosition(0.885);
    wait(1000);
    robot.turnToHeading(90, 0.5, super::opModeIsActive, super::processChanges);
    retreat();
  }

  private void retreat() {
    robot.drivetrain.driveBySticks(0.6, 0, 0);
    wait(700);
    robot.drivetrain.setAllPower(0);
    wait(500);
    robot.driveForward(.5, 1.5, -34, 20, super::opModeIsActive, super::processChanges);
    robot.drivetrain.driveBySticks(-0.6, 0, 0);
    wait(1200);
    robot.turnToHeading(0, 0.5, super::opModeIsActive, super::processChanges);
    robot.drivetrain.driveBySticks(.6, 0, 0);
    wait(1150);
    robot.driveForward(0.5, 0.5, -6, 20, super::opModeIsActive, super::processChanges);
    robot.intake.beginIntaking();
    robot.lift.setArmTwoPosition(TurretBotPosition.INTAKE_POSITION.secondJointTarget());
    wait(250);
    robot.intake.stop();
    robot.gripper.close();
    robot.firstJointAdjustment.getAndAdd(-200);
    robot.goToPosition(TurretBotPosition.INTAKE_POSITION, false);
    wait(4000);
    robot.gripper.open();
    robot.driveForward(0.5, 0.5, 9, 20, super::opModeIsActive, super::processChanges);
  }

  @Override
  public void onStop() {}
}
