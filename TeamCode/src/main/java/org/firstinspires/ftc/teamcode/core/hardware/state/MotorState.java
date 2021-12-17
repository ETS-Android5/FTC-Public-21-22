package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

public class MotorState extends IMotorState {
  private final Direction direction;
  private final RunMode runMode;
  private final ZeroPowerBehavior zeroPowerBehavior;
  private final double power;
  private final int targetPosition;
  private final TriFunction<Integer, Integer, Integer, Double> powerCurve;
  private final double maxAcceleration;

  public MotorState(String name, Direction direction) {
    this(name, direction, RunMode.RUN_WITHOUT_ENCODER, ZeroPowerBehavior.BRAKE, 0.0, 0, null, 0.0);
  }

  public MotorState(
      String name,
      Direction direction,
      RunMode runMode,
      ZeroPowerBehavior zeroPowerBehavior,
      Double power,
      Integer targetPosition,
      TriFunction<Integer, Integer, Integer, Double> powerCurve,
      Double maxAcceleration) {
    super(name);
    this.direction = direction;
    this.runMode = runMode;
    this.zeroPowerBehavior = zeroPowerBehavior;
    this.power = power;
    this.targetPosition = targetPosition;
    this.powerCurve = powerCurve;
    this.maxAcceleration = maxAcceleration;
  }

  @Override
  public Direction getDirection() {
    return direction;
  }

  @Override
  public IMotorState withDirection(Direction direction) {
    return new MotorState(
        this.name,
        direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
            this.maxAcceleration);
  }

  @Override
  public RunMode getRunMode() {
    return runMode;
  }

  @Override
  public IMotorState withRunMode(RunMode runMode) {
    return new MotorState(
        this.name,
        this.direction,
        runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
            this.maxAcceleration);
  }

  @Override
  public ZeroPowerBehavior getZeroPowerBehavior() {
    return zeroPowerBehavior;
  }

  @Override
  public IMotorState withZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
            this.maxAcceleration);
  }

  @Override
  public double getPower() {
    return power;
  }

  @Override
  public IMotorState withPower(double power) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        power < -1 ? -1 : power > 1 ? 1 : power,
        this.targetPosition,
        this.powerCurve,
            this.maxAcceleration);
  }

  @Override
  public int getTargetPosition() {
    return targetPosition;
  }

  @Override
  public IMotorState withTargetPosition(int targetPosition) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        targetPosition,
        this.powerCurve,
            this.maxAcceleration);
  }

  @Override
  public TriFunction<Integer, Integer, Integer, Double> getPowerCurve() {
    return this.powerCurve;
  }

  @Override
  public IMotorState withPowerCurve(TriFunction<Integer, Integer, Integer, Double> powerCurve) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        powerCurve,
            this.maxAcceleration);
  }

  @Override
  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  @Override
  public IMotorState withMaxAcceleration(double maxAcceleration) {
    return new MotorState(
            name, direction, runMode, zeroPowerBehavior, power, targetPosition, powerCurve, maxAcceleration
    );
  }

  @Override
  public IMotorState duplicate() {
    return new MotorState(
        name, direction, runMode, zeroPowerBehavior, power, targetPosition, powerCurve, maxAcceleration);
  }
}
