package org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class Intake implements IIntake<IntakeState> {
  private static final String INTAKE_MOTOR_NAME = "INTAKE_MOTOR";

  private static final double STOPPED_SPEED = 0.0;
  private static final double MOVING_SPEED = 1.0;
  private static final double INTAKING_SPEED = MOVING_SPEED;
  private static final double OUTTAKING_SPEED = MOVING_SPEED * -1;

  @Hardware(name = INTAKE_MOTOR_NAME)
  public DcMotor intakeMotor;

  private IMotorState intakeMotorState;

  public Intake() {
    initialize();
  }

  private void initialize() {
    intakeMotorState = new MotorState(INTAKE_MOTOR_NAME, Direction.FORWARD);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Collections.singletonList(intakeMotorState);
  }

  @Override
  @Observable(key = "INTAKE")
  public IntakeState getState() {
    return intakeMotorState.getPower() == INTAKING_SPEED
        ? IntakeState.INTAKING
        : intakeMotorState.getPower() == OUTTAKING_SPEED
            ? IntakeState.OUTTAKING
            : IntakeState.STOPPED;
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
  public synchronized void stop() {
    intakeMotorState = intakeMotorState.withPower(STOPPED_SPEED);
  }
}
