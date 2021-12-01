package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.IMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.ISingleJointAngularLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.SingleJointAngularLift;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class NewChassis implements Component {
  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final ISingleJointAngularLift lift = new SingleJointAngularLift();

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    List<? super State> states = new LinkedList<>();
    Arrays.asList(drivetrain, lift)
        .forEach((component) -> component.getNextState().forEach((s) -> states.add((State) s)));
    return states;
  }
}
