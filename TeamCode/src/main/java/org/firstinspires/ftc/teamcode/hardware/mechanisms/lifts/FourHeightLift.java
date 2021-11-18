package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class FourHeightLift implements IFourHeightLift {
  private static final String LIFT_MOTOR_NAME = "LIFT_MOTOR";

  private static final int HEIGHT_0_TICKS = 0;
  private static final int HEIGHT_1_TICKS = 800;
  private static final int HEIGHT_2_TICKS = 1260;
  private static final int HEIGHT_3_TICKS = 2300;

  public FourHeightLift() {
    initialize();
  }

  @Hardware(name = LIFT_MOTOR_NAME, runMode = RunMode.RUN_TO_POSITION)
  public DcMotor liftMotor;

  private IMotorState liftMotorState;

  private void initialize() {
    liftMotorState =
        new MotorState(LIFT_MOTOR_NAME, false)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(HEIGHT_0_TICKS)
            .withPowerCurve(PowerCurves.RUN_TO_POSITION_RAMP);
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
  @Observable(key = "LIFTSTATE")
  public FourHeightLiftState getState() {
    switch (liftMotorState.getTargetPosition()) {
      case HEIGHT_1_TICKS:
        return FourHeightLiftState.HEIGHT_1;
      case HEIGHT_2_TICKS:
        return FourHeightLiftState.HEIGHT_2;
      case HEIGHT_3_TICKS:
        return FourHeightLiftState.HEIGHT_3;
      default:
        return FourHeightLiftState.HEIGHT_0;
    }
  }

  @Override
  public void goToHeight0() {
    liftMotorState = liftMotorState.withTargetPosition(HEIGHT_0_TICKS);
  }

  @Override
  public void goToHeight1() {
    liftMotorState = liftMotorState.withTargetPosition(HEIGHT_1_TICKS);
  }

  @Override
  public void goToHeight2() {
    liftMotorState = liftMotorState.withTargetPosition(HEIGHT_2_TICKS);
  }

  @Override
  public void goToHeight3() {
    liftMotorState = liftMotorState.withTargetPosition(HEIGHT_3_TICKS);
  }
}
