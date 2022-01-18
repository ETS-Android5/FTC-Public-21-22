package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;

@Autonomous(name = "Smh")
public class Smh extends EnhancedAutonomous {
  private final TurretBot robot;

  public Smh() {
    super(new TurretBot(Alliance.RED, TurretBot.AUTO_FIRST_JOINT_OFFSET, true));
    this.robot = (TurretBot) super.robotObject;
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void onStartPressed() {
    robot.lift.setArmTwoPosition(0.8);
    int delay = robot.grabFreight();
    processChanges();
    sleep(delay);
    robot.lift.setArmOnePosition(robot.firstJointOffset + 550);
    long start1 = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start1 < 4000) {
      processChanges();
    }
    robot.lift.setArmTwoPosition(0.47);
    long start2 = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start2 < 2000) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0.5);
    processChanges();
    long start3 = System.currentTimeMillis();
    while (opModeIsActive() && System.currentTimeMillis() - start3 < 1250) {
      processChanges();
    }
    robot.drivetrain.setAllPower(0);
    processChanges();
    robot.goToPosition(TurretBotPosition.INTAKE_POSITION);
    while (opModeIsActive()) {
      processChanges();
    }
  }

  @Override
  public void onStop() {}
}
