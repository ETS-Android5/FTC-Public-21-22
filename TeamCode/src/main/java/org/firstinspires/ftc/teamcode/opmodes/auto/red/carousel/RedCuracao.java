package org.firstinspires.ftc.teamcode.opmodes.auto.red.carousel;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
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

@Autonomous(name = "CB_AUTO_RedCuracao")
@SuppressWarnings("unused")
public class RedCuracao extends EnhancedAutonomous {
  private final TurretBot robot;

  public RedCuracao() {
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
    robot.driveForward(.5, 0.5, -19, 20, super::opModeIsActive, super::processChanges);
    switch (teamMarkerPosition.get()) {
      case LEFT:
        Log.d("TURRETBOT", "SAW LEFT");
        robot.turret.turnCCWToBack();
        wait(500);
        robot.lift.setArmOnePosition(TurretBotPosition.ALLIANCE_BOTTOM_POSITION.firstJointTarget() - 6);
        robot.lift.setArmTwoPosition(TurretBotPosition.ALLIANCE_BOTTOM_POSITION.secondJointTarget() + .03);
        wait(3000);
        robot.turret.turnToPosition(TurretBotPosition.AUTO_REACH_BOTTOM.turretTarget(Alliance.BLUE));
        break;
      case CENTER:
        Log.d("TURRETBOT", "SAW CENTER");
        robot.turret.turnCCWToBack();
        wait(500);
        robot.lift.setArmOnePosition(TurretBotPosition.ALLIANCE_MIDDLE_POSITION.firstJointTarget() - 10);
        robot.lift.setArmTwoPosition(TurretBotPosition.ALLIANCE_MIDDLE_POSITION.secondJointTarget() + .02);
        wait(2000);
        robot.turret.turnToPosition(TurretBotPosition.AUTO_REACH_MIDDLE.turretTarget(Alliance.BLUE) + (2 * Turret.DEGREES_TO_TICKS));
        break;
      case RIGHT:
        Log.d("TURRETBOT", "SAW RIGHT");
        robot.turret.turnCCWToBack();
        wait(500);
        robot.lift.setArmOnePosition(TurretBotPosition.ALLIANCE_TOP_POSITION.firstJointTarget() - 320);
        robot.lift.setArmTwoPosition(TurretBotPosition.ALLIANCE_TOP_POSITION.secondJointTarget() - 0.2);
        wait(1500);
        robot.turret.turnToPosition(TurretBotPosition.AUTO_REACH_TOP.turretTarget(Alliance.BLUE) + (10 * Turret.DEGREES_TO_TICKS));
        break;
    }
    wait(1000);
    theRestOfCuracao();
  }

  private void theRestOfCuracao() {
    wait(robot.dropFreight());
    robot.drivetrain.driveBySticks(0.6, 0.13, 0);
    wait(700);
    robot.drivetrain.setAllPower(0);
    robot.turret.turnCCWToBack();
    wait(1000);
    robot.lift.setArmOnePosition(570);
    wait(1000);
    robot.lift.setArmTwoPosition(0.885);
    wait(1000);
    robot.turret.turnToPosition(robot.turretAdjustment.get());
    robot.driveForward(.5, 0.5, 12, 20, super::opModeIsActive, super::processChanges);
    robot.turnToHeading(95, 0.5, super::opModeIsActive, super::processChanges);
    wait(300);
    robot.turnToHeading(95, 0.5, super::opModeIsActive, super::processChanges);
    retreat();
  }

  private void retreat() {
    robot.driveForward(.35, 1.5, -19, 20, super::opModeIsActive, super::processChanges);
    wait(500);
    robot.turnToHeading(95, 0.5, PowerCurves.generatePowerCurve(.3, .3), super::opModeIsActive, super::processChanges);
    robot.carouselSpinner.spinForward();
    wait(4250);
    robot.drivetrain.driveBySticks(-0.6, 0, 0);
    wait(1000);
    robot.driveForward(.3, 1.5, -5, 20, super::opModeIsActive, super::processChanges);
    robot.intake.beginIntaking();
    robot.lift.setArmTwoPosition(TurretBotPosition.INTAKE_POSITION.secondJointTarget());
    wait(250);
    robot.intake.stop();
    robot.gripper.close();
    robot.firstJointAdjustment.getAndAdd(-200);
    robot.goToPosition(TurretBotPosition.INTAKE_POSITION, false);
    wait(4000);
    robot.gripper.open();
    wait(4000);
  }

  @Override
  public void onStop() {}
}
