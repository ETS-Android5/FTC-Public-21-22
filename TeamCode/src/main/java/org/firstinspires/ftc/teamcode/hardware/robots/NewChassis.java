package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ICarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ISingleServoGripper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ITurret;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.SingleServoGripper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.IMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.IIntake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.IntakeState;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.DualJointAngularLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.IDualJointAngularLift;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class NewChassis implements Component {
  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IIntake<IntakeState> intake = new Intake();
  public final ICarouselSpinner carouselSpinner = new CarouselSpinner();
  public final ITurret turret = new Turret();
  public final IDualJointAngularLift lift = new DualJointAngularLift();
  public final ISingleServoGripper gripper = new SingleServoGripper();

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    List<? super State> states = new LinkedList<>();
    Arrays.asList(drivetrain, intake, carouselSpinner, turret, lift, gripper)
        .forEach((component) -> component.getNextState().forEach((s) -> states.add((State) s)));
    return states;
  }
}
