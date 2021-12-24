package org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;

public class RampedIntake implements IRampedIntake {
  private static final String INTAKE_MOTOR_NAME = "INTAKE_MOTOR";
  private static final String INTAKE_SERVO_NAME = "INTAKE_SERVO";

  private static final double STOPPED_SPEED = 0.0;
  private static final double MOVING_SPEED = 1.0;
  private static final double INTAKING_SPEED = MOVING_SPEED;
  private static final double OUTTAKING_SPEED = MOVING_SPEED * -1;
  private static final double SLOWLY_OUTTAKING_SPEED = -0.45;
  private static final double LOWERED_POSITION = 0.33;
  private static final double RAISED_POSITION = 0.0;

  @Hardware(name = INTAKE_MOTOR_NAME)
  public DcMotor intakeMotor;

  @Hardware(name = INTAKE_SERVO_NAME)
  public Servo intakeServo;

  private IMotorState intakeMotorState;
  private IServoState intakeServoState;

  public RampedIntake() {
    initialize();
  }

  private void initialize() {
    intakeMotorState = new MotorState(INTAKE_MOTOR_NAME, Direction.FORWARD);
    intakeServoState = new ServoState(INTAKE_SERVO_NAME, Direction.FORWARD, LOWERED_POSITION);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Arrays.asList(intakeMotorState, intakeServoState);
  }

  @Override
  public synchronized void raise() {
    intakeServoState = intakeServoState.withPosition(RAISED_POSITION);
  }

  @Override
  public synchronized void lower() {
    intakeServoState = intakeServoState.withPosition(LOWERED_POSITION);
  }

  @Override
  public void toggleIntaking() {
    RampedIntakeState state = getState();
    if (state != RampedIntakeState.LOWERED_AND_INTAKING
        && state != RampedIntakeState.RAISED_AND_INTAKING) {
      beginIntaking();
    } else {
      stop();
    }
  }

  @Override
  public void toggleOuttaking() {
    RampedIntakeState state = getState();
    if (state != RampedIntakeState.LOWERED_AND_OUTTAKING
        && state != RampedIntakeState.RAISED_AND_OUTTAKING) {
      beginOuttaking();
    } else {
      stop();
    }
  }

  @Override
  public synchronized void beginIntaking() {
    intakeMotorState = intakeMotorState.withPower(INTAKING_SPEED);
  }

  @Override
  public synchronized void beginOuttaking() {
    intakeMotorState = intakeMotorState.withPower(OUTTAKING_SPEED);
  }

  @Override
  public void beginOuttakingSlowly() {
    intakeMotorState = intakeMotorState.withPower(SLOWLY_OUTTAKING_SPEED);
  }

  @Override
  public synchronized void stop() {
    intakeMotorState = intakeMotorState.withPower(STOPPED_SPEED);
  }

  @Override
  @Observable(key = "RAMPED_INTAKE")
  public RampedIntakeState getState() {
    double power = intakeMotorState.getPower();
    if (intakeServoState.getPosition() == LOWERED_POSITION) {
      if (power == INTAKING_SPEED) {
        return RampedIntakeState.LOWERED_AND_INTAKING;
      }
      if (power == OUTTAKING_SPEED) {
        return RampedIntakeState.LOWERED_AND_OUTTAKING;
      }
      if (power == SLOWLY_OUTTAKING_SPEED) {
        return RampedIntakeState.LOWERED_AND_OUTTAKING_SLOWLY;
      }
      return RampedIntakeState.LOWERED_AND_STOPPED;
    } else {
      if (power == INTAKING_SPEED) {
        return RampedIntakeState.RAISED_AND_INTAKING;
      }
      if (power == OUTTAKING_SPEED) {
        return RampedIntakeState.RAISED_AND_OUTTAKING;
      }
      if (power == SLOWLY_OUTTAKING_SPEED) {
        return RampedIntakeState.RAISED_AND_OUTTAKING_SLOWLY;
      }
      return RampedIntakeState.RAISED_AND_STOPPED;
    }
  }
}
