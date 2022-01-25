package org.firstinspires.ftc.teamcode.opmodes.auto.red.carousel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.DualJointAngularLift;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;

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
    robot.lift.setArmTwoPosition(0.86);
    int delay = robot.grabFreight();
    processChanges();
    sleep(delay);
    robot.lift.setArmOnePosition(robot.firstJointOffset + 550);
    processChanges();
    robot.drivetrain.setAllPower(0.25);
    double startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive() && robot.drivetrain.avgEncoderValue() - startAvg <= 700) {
      processChanges();
    }
    robot.drivetrain.driveBySticks(-0.5, 0, 0);
    long strafeStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - strafeStart < 750) {
      processChanges();
    }
    robot.drivetrain.setAllPower(-0.25);
    robot.lift.setArmTwoPosition(0.59);
    startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive() && startAvg - robot.drivetrain.avgEncoderValue() <= 430) {
      processChanges();
    }
    robot.drivetrain.setAllPower(-0.02);
    robot.lift.setArmOnePosition(-130);
    processChanges();
    robot.carouselSpinner.spinForward();
    long spinStart = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - spinStart < 4000) {
      processChanges();
    }
    robot.lift.setArmTwoPosition(DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
    robot.drivetrain.setAllPower(0.25);
    startAvg = robot.drivetrain.avgEncoderValue();
    while (opModeIsActive() && robot.drivetrain.avgEncoderValue() - startAvg <= 640) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
  }

  @Override
  public void onStop() {}
}
