package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class SingleJointAngularLift implements ISingleJointAngularLift {
  private static final String LIFT_MOTOR_NAME = "LIFT_MOTOR";

  @Hardware(name = LIFT_MOTOR_NAME, runMode = RunMode.RUN_TO_POSITION)
  public DcMotor liftMotor;

  private IMotorState liftMotorState;

  public SingleJointAngularLift() {
    initialize();
  }

  private void initialize() {
    liftMotorState =
        new MotorState(LIFT_MOTOR_NAME, Direction.FORWARD)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(0)
            .withPowerCurve(PowerCurves.RUN_TO_POSITION_ANGULAR_LIFT_CURVE);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Collections.singletonList(liftMotorState.duplicate());
  }

  @Override
  @Observable(key = "SINGLE_JOINT_ANGULAR_LIFT")
  public Integer getState() {
    return liftMotorState.getTargetPosition();
  }

  @Override
  public synchronized void goToPosition(int ticks) {
    liftMotorState = liftMotorState.withTargetPosition(ticks);
  }
}
