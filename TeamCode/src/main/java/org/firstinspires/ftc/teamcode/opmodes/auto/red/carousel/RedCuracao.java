package org.firstinspires.ftc.teamcode.opmodes.auto.red.carousel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;

@Autonomous(name = "CB_AUTO_RedCuracao")
@SuppressWarnings("unused")
public class RedCuracao extends EnhancedAutonomous {
  private final TurretBot robot;

  public RedCuracao() {
    super(new TurretBot(Alliance.RED, TurretBot.AUTO_FIRST_JOINT_OFFSET, true));
    this.robot = (TurretBot) super.robotObject;
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void onStartPressed() {
    robot.rearRange.startSampling();
    robot.gyro.startSampling();
    robot.drivetrain.setRunMode(RunMode.RUN_USING_ENCODER);
    robot.lift.setArmTwoPosition(0.8);
    int delay = robot.grabFreight();
    processChanges();
    sleep(delay);
    robot.lift.setArmOnePosition(robot.firstJointOffset + 550);
    processChanges();
    robot.drivetrain.setAllPower(-0.25);
    while (opModeIsActive()
        && InterpolatablePipe.getInstance().currentDataPointOf(robot.rearRange.getName()) <= 400) {
      processChanges();
    }
    robot.drivetrain.driveBySticks(-0.5, 0, 0);
    long strafeStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - strafeStart < 750) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0.25);
    robot.lift.setArmTwoPosition(0.47);
    long revStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - revStart < 1000) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0.05);
    processChanges();
    robot.carouselSpinner.spinBackward();
    robot.grabTeamMarker();
    robot.goToPosition(TurretBotPosition.INTAKE_POSITION);
    long spinStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - spinStart < 5000) {
      processChanges();
    }
    robot.drivetrain.setAllPower(-0.25);
    long forwardStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - forwardStart < 1500) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
  }

  @Override
  public void onStop() {}
}
