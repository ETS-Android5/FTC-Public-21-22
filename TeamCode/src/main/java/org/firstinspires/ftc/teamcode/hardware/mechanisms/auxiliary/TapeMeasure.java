package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;

import kotlin.Triple;

public class TapeMeasure implements ITapeMeasure {
  private static final String YAW_SERVO_NAME = "TAPE_MEASURE_YAW_SERVO";
  private static final String PITCH_SERVO_NAME = "TAPE_MEASURE_PITCH_SERVO";
  private static final String LENGTH_SERVO_NAME = "TAPE_MEASURE_LENGTH_SERVO";
  private static final double SERVO_INIT_SPEED = 0;
  private static final double ADJUSTMENT_RATE = 0.001;

  @Hardware(name = YAW_SERVO_NAME)
  @SuppressWarnings("unused")
  public Servo yawServo;

  @Hardware(name = PITCH_SERVO_NAME)
  @SuppressWarnings("unused")
  public Servo pitchServo;

  @Hardware(name = LENGTH_SERVO_NAME)
  @SuppressWarnings("unused")
  public CRServo lengthServo;

  private IServoState yawServoState;
  private IServoState pitchServoState;
  private IServoState lengthServoState;

  public TapeMeasure() {
    initialize();
  }

  private void initialize() {
    yawServoState =
        new ServoState(YAW_SERVO_NAME, Direction.FORWARD, SERVO_INIT_SPEED, false)
            .withPosition(0.56);
    pitchServoState =
        new ServoState(PITCH_SERVO_NAME, Direction.FORWARD, SERVO_INIT_SPEED, false)
            .withPosition(0.5);
    lengthServoState = new ServoState(LENGTH_SERVO_NAME, Direction.FORWARD, SERVO_INIT_SPEED, true);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Arrays.asList(yawServoState, pitchServoState, lengthServoState);
  }

  @Override
  public Triple<Double, Double, Double> getState() {
    return new Triple<>(
        yawServoState.getPosition(), pitchServoState.getPosition(), lengthServoState.getPosition());
  }

  @Override
  public synchronized void adjustYaw(double amt) {
    double position = yawServoState.getPosition() - (amt * ADJUSTMENT_RATE);
    position = position < 0 ? 0 : position > 1 ? 1 : position;
    yawServoState = yawServoState.withPosition(position);
  }

  @Override
  public synchronized void adjustPitch(double amt) {
    double position = pitchServoState.getPosition() - (amt * ADJUSTMENT_RATE);
    position = position < 0 ? 0 : position > 1 ? 1 : position;
    pitchServoState = pitchServoState.withPosition(position);
  }

  @Override
  public synchronized void setLengthRate(double amt) {
    lengthServoState = lengthServoState.withPosition(amt < -1 ? -1 : amt > 1 ? 1 : amt);
  }
}
