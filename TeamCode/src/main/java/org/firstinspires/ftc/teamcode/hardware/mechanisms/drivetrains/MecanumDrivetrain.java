package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.CallbackData;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class MecanumDrivetrain implements IMecanumDrivetrain {
  private static final String FRONT_LEFT_MOTOR_NAME = "FRONT_LEFT_MOTOR";
  private static final String FRONT_RIGHT_MOTOR_NAME = "FRONT_RIGHT_MOTOR";
  private static final String REAR_LEFT_MOTOR_NAME = "REAR_LEFT_MOTOR";
  private static final String REAR_RIGHT_MOTOR_NAME = "REAR_RIGHT_MOTOR";

  private static final double THEORETICAL_TICKS_PER_INCH = 43.4734038779;
  private static final double DRIVETRAIN_MECHANICAL_EFFICIENCY_COEFFICIENT = 1;
  public static final double TICKS_PER_INCH =
      THEORETICAL_TICKS_PER_INCH * DRIVETRAIN_MECHANICAL_EFFICIENCY_COEFFICIENT;

  @Hardware(
      name = FRONT_LEFT_MOTOR_NAME,
      direction = Direction.REVERSE,
      runMode = RunMode.RUN_USING_ENCODER)
  @SuppressWarnings("unused")
  public DcMotorEx frontLeftMotor;

  @Hardware(
      name = FRONT_RIGHT_MOTOR_NAME,
      direction = Direction.FORWARD,
      runMode = RunMode.RUN_USING_ENCODER)
  @SuppressWarnings("unused")
  public DcMotorEx frontRightMotor;

  @Hardware(
      name = REAR_LEFT_MOTOR_NAME,
      direction = Direction.FORWARD,
      runMode = RunMode.RUN_USING_ENCODER)
  @SuppressWarnings("unused")
  public DcMotorEx rearLeftMotor;

  @Hardware(
      name = REAR_RIGHT_MOTOR_NAME,
      direction = Direction.FORWARD,
      runMode = RunMode.RUN_USING_ENCODER)
  @SuppressWarnings("unused")
  public DcMotorEx rearRightMotor;

  protected IMotorState frontLeftMotorState;
  protected IMotorState frontRightMotorState;
  protected IMotorState rearLeftMotorState;
  protected IMotorState rearRightMotorState;
  private boolean autoMode = false;

  public MecanumDrivetrain() {
    initialize();
  }

  private void initialize() {
    frontLeftMotorState =
        new MotorState(FRONT_LEFT_MOTOR_NAME, Direction.REVERSE)
            .withRunMode(RunMode.RUN_USING_ENCODER);
    frontRightMotorState =
        new MotorState(FRONT_RIGHT_MOTOR_NAME, Direction.FORWARD)
            .withRunMode(RunMode.RUN_USING_ENCODER);
    rearLeftMotorState =
        new MotorState(REAR_LEFT_MOTOR_NAME, Direction.REVERSE)
            .withRunMode(RunMode.RUN_USING_ENCODER);
    rearRightMotorState =
        new MotorState(REAR_RIGHT_MOTOR_NAME, Direction.FORWARD)
            .withRunMode(RunMode.RUN_USING_ENCODER);
  }

  @Override
  public synchronized void setAllPower(double power) {
    frontLeftMotorState = frontLeftMotorState.withPower(power);
    frontRightMotorState = frontRightMotorState.withPower(power);
    rearLeftMotorState = rearLeftMotorState.withPower(power);
    rearRightMotorState = rearRightMotorState.withPower(power);
  }

  @Override
  public synchronized void setLeftPower(double power) {
    frontLeftMotorState = frontLeftMotorState.withPower(power);
    rearLeftMotorState = rearLeftMotorState.withPower(power);
  }

  @Override
  public synchronized void setRightPower(double power) {
    frontRightMotorState = frontRightMotorState.withPower(power);
    rearRightMotorState = rearRightMotorState.withPower(power);
  }

  @Override
  public synchronized void setAllTarget(int ticks) {
    frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
    frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
    rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
    rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
  }

  @Override
  public synchronized void setLeftTarget(int ticks) {
    frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
    rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
  }

  @Override
  public synchronized void setRightTarget(int ticks) {
    frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
    rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
  }

  @Override
  public void reset() {
    initialize();
  }

  @Override
  public synchronized void setRunMode(RunMode runMode) {
    autoMode = runMode == RunMode.RUN_TO_POSITION;
    frontLeftMotorState = frontLeftMotorState.withRunMode(runMode);
    frontRightMotorState = frontRightMotorState.withRunMode(runMode);
    rearLeftMotorState = rearLeftMotorState.withRunMode(runMode);
    rearRightMotorState = rearRightMotorState.withRunMode(runMode);
  }

  @Override
  public double avgEncoderValue() {
    return (MotorTrackerPipe.getInstance().getPositionOf(FRONT_LEFT_MOTOR_NAME)
            + MotorTrackerPipe.getInstance().getPositionOf(FRONT_RIGHT_MOTOR_NAME)
            + MotorTrackerPipe.getInstance().getPositionOf(REAR_LEFT_MOTOR_NAME)
            + MotorTrackerPipe.getInstance().getPositionOf(REAR_RIGHT_MOTOR_NAME))
        / 4.0;
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return autoMode
        ? Arrays.asList(
            frontLeftMotorState.duplicate(),
            frontRightMotorState.duplicate(),
            rearLeftMotorState.duplicate(),
            rearRightMotorState.duplicate())
        : Arrays.asList(
            frontLeftMotorState, frontRightMotorState, rearLeftMotorState, rearRightMotorState);
  }

  @Override
  public Direction getFrontLeftDirection() {
    return frontLeftMotorState.getDirection();
  }

  @Override
  public synchronized void setFrontLeftDirection(Direction direction) {
    frontLeftMotorState = frontLeftMotorState.withDirection(direction);
  }

  @Override
  public Direction getFrontRightDirection() {
    return frontRightMotorState.getDirection();
  }

  @Override
  public synchronized void setFrontRightDirection(Direction direction) {
    frontRightMotorState = frontRightMotorState.withDirection(direction);
  }

  @Override
  public Direction getRearLeftDirection() {
    return rearLeftMotorState.getDirection();
  }

  @Override
  public synchronized void setRearLeftDirection(Direction direction) {
    rearLeftMotorState = rearLeftMotorState.withDirection(direction);
  }

  @Override
  public Direction getRearRightDirection() {
    return rearRightMotorState.getDirection();
  }

  @Override
  public synchronized void setRearRightDirection(Direction direction) {
    rearRightMotorState = rearRightMotorState.withDirection(direction);
  }

  @Override
  public synchronized void setFrontLeftPower(double power) {
    frontLeftMotorState = frontLeftMotorState.withPower(power);
  }

  @Override
  public synchronized void setFrontRightPower(double power) {
    frontRightMotorState = frontRightMotorState.withPower(power);
  }

  @Override
  public synchronized void setRearLeftPower(double power) {
    rearLeftMotorState = rearLeftMotorState.withPower(power);
  }

  @Override
  public synchronized void setRearRightPower(double power) {
    rearRightMotorState = rearRightMotorState.withPower(power);
  }

  @Override
  public synchronized void setFrontLeftTarget(int ticks) {
    frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
  }

  @Override
  public synchronized void setFrontRightTarget(int ticks) {
    frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
  }

  @Override
  public synchronized void setRearLeftTarget(int ticks) {
    rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
  }

  @Override
  public synchronized void setRearRightTarget(int ticks) {
    rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
  }

  @Override
  public void driveBySticks(double lateral, double longitudinal, double turn) {
    double wheelPower = Math.hypot(lateral, longitudinal);
    double stickAngleRadians = Math.atan2(longitudinal, lateral) - Math.PI / 4;

    double sinAngleRadians = Math.sin(stickAngleRadians);
    double cosAngleRadians = Math.cos(stickAngleRadians);

    double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

    synchronized (this) {
      frontLeftMotorState =
          frontLeftMotorState.withPower(wheelPower * cosAngleRadians * factor + turn);
      frontRightMotorState =
          frontRightMotorState.withPower(wheelPower * sinAngleRadians * factor - turn);
      rearLeftMotorState =
          rearLeftMotorState.withPower(wheelPower * sinAngleRadians * factor + turn);
      rearRightMotorState =
          rearRightMotorState.withPower(wheelPower * cosAngleRadians * factor - turn);
    }
  }

  @Override
  public synchronized void setPowerCurve(TriFunction<Double, Double, Double, Double> powerCurve) {
    frontLeftMotorState = frontLeftMotorState.withPowerCurve(powerCurve);
    frontRightMotorState = frontRightMotorState.withPowerCurve(powerCurve);
    rearLeftMotorState = rearLeftMotorState.withPowerCurve(powerCurve);
    rearRightMotorState = rearRightMotorState.withPowerCurve(powerCurve);
  }

  @Override
  public void autoRunToPosition(
      int target, int toleranceTicks, Supplier<Boolean> opModeIsActive, Runnable update) {
    autoRunToPosition(target, target, target, target, toleranceTicks, opModeIsActive, update);
  }

  @Override
  public void autoRunToPosition(
      int leftTicks,
      int rightTicks,
      int toleranceTicks,
      Supplier<Boolean> opModeIsActive,
      Runnable update) {
    autoRunToPosition(
        leftTicks, rightTicks, leftTicks, rightTicks, toleranceTicks, opModeIsActive, update);
  }

  @Override
  public void autoRunToPosition(
      int frontLeftTicks,
      int frontRightTicks,
      int rearLeftTicks,
      int rearRightTicks,
      int toleranceTicks,
      Supplier<Boolean> opModeIsActive,
      Runnable update) {
    boolean[] currentTargetsReached = new boolean[4];
    setFrontLeftTarget(frontLeftTicks);
    setFrontRightTarget(frontRightTicks);
    setRearLeftTarget(rearLeftTicks);
    setRearRightTarget(rearRightTicks);
    MotorTrackerPipe.getInstance()
        .setCallbackForMotorPosition(
            new CallbackData<>(
                getFrontLeftName(),
                (Integer i) ->
                    i != null
                        && i >= frontLeftTicks - toleranceTicks
                        && i <= frontLeftTicks + toleranceTicks,
                () -> currentTargetsReached[0] = true));
    MotorTrackerPipe.getInstance()
        .setCallbackForMotorPosition(
            new CallbackData<>(
                getFrontRightName(),
                (Integer i) ->
                    i != null
                        && i >= frontRightTicks - toleranceTicks
                        && i <= frontLeftTicks + toleranceTicks,
                () -> currentTargetsReached[1] = true));
    MotorTrackerPipe.getInstance()
        .setCallbackForMotorPosition(
            new CallbackData<>(
                getRearLeftName(),
                (Integer i) ->
                    i != null
                        && i >= rearLeftTicks - toleranceTicks
                        && i <= frontLeftTicks + toleranceTicks,
                () -> currentTargetsReached[2] = true));
    MotorTrackerPipe.getInstance()
        .setCallbackForMotorPosition(
            new CallbackData<>(
                getRearRightName(),
                (Integer i) ->
                    i != null
                        && i >= rearRightTicks - toleranceTicks
                        && i <= frontLeftTicks + toleranceTicks,
                () -> currentTargetsReached[3] = true));
    update.run();
    while (opModeIsActive.get()
        && (!currentTargetsReached[0]
            || !currentTargetsReached[1]
            || !currentTargetsReached[2]
            || !currentTargetsReached[3])) {
      update.run();
    }
  }

  @Override
  public String getFrontLeftName() {
    return frontLeftMotorState.getName();
  }

  @Override
  public String getFrontRightName() {
    return frontRightMotorState.getName();
  }

  @Override
  public String getRearLeftName() {
    return rearLeftMotorState.getName();
  }

  @Override
  public String getRearRightName() {
    return rearRightMotorState.getName();
  }
}
