package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.hardware.robots.SliderLiftBot;

@Autonomous(name = "CB_AUTO_StraightLine")
@Disabled
public class StraightLine extends EnhancedAutonomous {

  private final SliderLiftBot robot;

  public StraightLine() {
    super(new SliderLiftBot());
    this.robot = (SliderLiftBot) robotObject;
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void onStartPressed() {
    processChanges();
    robot.drivetrain.setAllPower(0.25);
    processChanges();
    sleep(2875);
    robot.drivetrain.setAllPower(0);
    processChanges();
  }

  @Override
  public void onStop() {}
}
