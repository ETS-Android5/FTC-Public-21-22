package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.LazyInject;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.core.magic.runtime.HardwareMapDependentReflectionBasedMagicRuntime;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import kotlin.Triple;

public class TapeMeasure implements ITapeMeasure {
  public static final String YAW_SERVO_NAME = "TAPE_MEASURE_YAW_SERVO";
  public static final String PITCH_SERVO_NAME = "TAPE_MEASURE_PITCH_SERVO";
  public static final String LENGTH_SERVO_NAME = "TAPE_MEASURE_LENGTH_SERVO";
  private static final double SERVO_INIT_SPEED = 0;
  private static final double ADJUSTMENT_RATE = 0.0007;

  private static final double YAW_MIN = 0;
  private static final double YAW_MAX = 0.63;
  private static final double PITCH_MIN = 0;
  private static final double PITCH_MAX = 0.62;

  private final AtomicBoolean initialized = new AtomicBoolean(false);

  @Hardware(name = YAW_SERVO_NAME)
  @LazyInject
  @SuppressWarnings("unused")
  public Servo yawServo;

  @Hardware(name = PITCH_SERVO_NAME)
  @LazyInject
  @SuppressWarnings("unused")
  public Servo pitchServo;

  @Hardware(name = LENGTH_SERVO_NAME)
  @LazyInject
  @SuppressWarnings("unused")
  public CRServo lengthServo;

  private IServoState yawServoState;
  private IServoState pitchServoState;
  private IServoState lengthServoState;
  private HardwareMapDependentReflectionBasedMagicRuntime runtime;

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
  @Observable(key = "TAPE MEASURE")
  public Triple<Double, Double, Double> getState() {
    return new Triple<>(
        yawServoState.getPosition(), pitchServoState.getPosition(), lengthServoState.getPosition());
  }

  @Override
  public synchronized void adjustYaw(double amt) {
    ensureInitialized();
    double position = yawServoState.getPosition() - (amt * ADJUSTMENT_RATE);
    position = position < YAW_MIN ? YAW_MIN : Math.min(position, YAW_MAX);
    yawServoState = yawServoState.withPosition(position);
  }

  @Override
  public synchronized void adjustPitch(double amt) {
    ensureInitialized();
    double position = pitchServoState.getPosition() - (amt * ADJUSTMENT_RATE);
    position = position < PITCH_MIN ? PITCH_MIN : Math.min(position, PITCH_MAX);
    pitchServoState = pitchServoState.withPosition(position);
  }

  @Override
  public synchronized void setYaw(double yaw) {
    ensureInitialized();
    yawServoState = yawServoState.withPosition(yaw < YAW_MIN ? YAW_MIN : Math.min(yaw, YAW_MAX));
  }

  @Override
  public synchronized void setPitch(double pitch) {
    ensureInitialized();
    pitchServoState =
        pitchServoState.withPosition(pitch < PITCH_MIN ? PITCH_MIN : Math.min(pitch, PITCH_MAX));
  }

  @Override
  public synchronized void setLengthRate(double amt) {
    ensureInitialized();
    lengthServoState = lengthServoState.withPosition(amt < -1 ? -1 : amt > 1 ? 1 : amt);
  }

  @Override
  public synchronized void setInitRuntime(HardwareMapDependentReflectionBasedMagicRuntime runtime) {
    this.runtime = runtime;
  }

  private void ensureInitialized() {
    if (runtime != null && !initialized.getAndSet(true)) {
      runtime.lateInit(TapeMeasure.YAW_SERVO_NAME);
      runtime.lateInit(TapeMeasure.PITCH_SERVO_NAME);
      runtime.lateInit(TapeMeasure.LENGTH_SERVO_NAME);
    }
  }
}
