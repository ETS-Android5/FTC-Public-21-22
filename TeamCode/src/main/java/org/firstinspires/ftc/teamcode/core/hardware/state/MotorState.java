package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.core.fn.QuadFunction;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

import java.util.function.Function;

public class MotorState extends IMotorState {
  private final Direction direction;
  private final RunMode runMode;
  private final ZeroPowerBehavior zeroPowerBehavior;
  private final double power;
  private final int targetPosition;
  private final TriFunction<Double, Double, Double, Double> powerCurve;
  private final int adjustmentThreshold;
  private final TriFunction<Double, Double, Double, Double> adjustmentCurve;
  private final TriFunction<Double, Double, Double, Double> adjustmentPowerCorrectionCurve;
  private final Function<Double, Double> powerAndTickRateRelation;
  private final TriFunction<Double, Double, Double, Double> powerCorrection;
  private final QuadFunction<Double, Double, Double, Double, Double> advPowerCorrection;
  private final String encoderDataSource;

  public MotorState(String name, Direction direction) {
    this(
        name,
        direction,
        RunMode.RUN_WITHOUT_ENCODER,
        ZeroPowerBehavior.BRAKE,
        0.0,
        0,
        null,
        0,
        null,
        null,
        null,
        null,
        null,
        null);
  }

  public MotorState(
      String name,
      Direction direction,
      RunMode runMode,
      ZeroPowerBehavior zeroPowerBehavior,
      Double power,
      Integer targetPosition,
      TriFunction<Double, Double, Double, Double> powerCurve,
      int adjustmentThreshold,
      TriFunction<Double, Double, Double, Double> adjustmentCurve,
      TriFunction<Double, Double, Double, Double> adjustmentPowerCorrectionCurve,
      Function<Double, Double> powerAndTickRateRelation,
      TriFunction<Double, Double, Double, Double> powerCorrection,
      QuadFunction<Double, Double, Double, Double, Double> advPowerCorrection,
      String encoderDataSource) {
    super(name);
    this.direction = direction;
    this.runMode = runMode;
    this.zeroPowerBehavior = zeroPowerBehavior;
    this.power = power;
    this.targetPosition = targetPosition;
    this.powerCurve = powerCurve;
    this.adjustmentThreshold = adjustmentThreshold;
    this.adjustmentCurve = adjustmentCurve;
    this.adjustmentPowerCorrectionCurve = adjustmentPowerCorrectionCurve;
    this.powerAndTickRateRelation = powerAndTickRateRelation;
    this.powerCorrection = powerCorrection;
    this.advPowerCorrection = advPowerCorrection;
    this.encoderDataSource = encoderDataSource;
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
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
        this.advPowerCorrection,
        this.encoderDataSource);
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
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
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
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
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
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
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
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public TriFunction<Double, Double, Double, Double> getPowerCurve() {
    return powerCurve;
  }

  @Override
  public IMotorState withPowerCurve(TriFunction<Double, Double, Double, Double> powerCurve) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        powerCurve,
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public int getAdjustmentThreshold() {
    return adjustmentThreshold;
  }

  @Override
  public IMotorState withAdjustmentThreshold(int adjustmentThreshold) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
        adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public TriFunction<Double, Double, Double, Double> getAdjustmentCurve() {
    return adjustmentCurve;
  }

  @Override
  public IMotorState withAdjustmentCurve(
      TriFunction<Double, Double, Double, Double> adjustmentCurve) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
        this.adjustmentThreshold,
        adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public TriFunction<Double, Double, Double, Double> getAdjustmentPowerCorrectionCurve() {
    return adjustmentPowerCorrectionCurve;
  }

  @Override
  public IMotorState withAdjustmentPowerCorrectionCurve(
      TriFunction<Double, Double, Double, Double> adjustmentPowerCorrectionCurve) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
        this.adjustmentThreshold,
        this.adjustmentCurve,
        adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public Function<Double, Double> getPowerAndTickRateRelation() {
    return powerAndTickRateRelation;
  }

  @Override
  public IMotorState withPowerAndTickRateRelation(
      Function<Double, Double> powerAndTickRateRelation) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public TriFunction<Double, Double, Double, Double> getPowerCorrection() {
    return powerCorrection;
  }

  @Override
  public IMotorState withPowerCorrection(
      TriFunction<Double, Double, Double, Double> powerCorrection) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        powerCorrection,
            this.advPowerCorrection,
        this.encoderDataSource);
  }

  @Override
  public QuadFunction<Double, Double, Double, Double, Double> getAdvPowerCorrection() {
    return advPowerCorrection;
  }

  @Override
  public IMotorState withAdvPowerCorrection(QuadFunction<Double, Double, Double, Double, Double> advPowerCorrection) {
    return new MotorState(
            this.name,
            this.direction,
            this.runMode,
            this.zeroPowerBehavior,
            this.power,
            this.targetPosition,
            this.powerCurve,
            this.adjustmentThreshold,
            this.adjustmentCurve,
            this.adjustmentPowerCorrectionCurve,
            this.powerAndTickRateRelation,
            this.powerCorrection,
            advPowerCorrection,
            this.encoderDataSource);
  }

  @Override
  public String getEncoderDataSource() {
    return encoderDataSource;
  }

  @Override
  public IMotorState withEncoderDataSource(String encoderDataSource) {
    return new MotorState(
        this.name,
        this.direction,
        this.runMode,
        this.zeroPowerBehavior,
        this.power,
        this.targetPosition,
        this.powerCurve,
        this.adjustmentThreshold,
        this.adjustmentCurve,
        this.adjustmentPowerCorrectionCurve,
        this.powerAndTickRateRelation,
        this.powerCorrection,
            this.advPowerCorrection,
        encoderDataSource);
  }

  @Override
  public IMotorState duplicate() {
    return new MotorState(
        name,
        direction,
        runMode,
        zeroPowerBehavior,
        power,
        targetPosition,
        powerCurve,
        adjustmentThreshold,
        adjustmentCurve,
        adjustmentPowerCorrectionCurve,
        powerAndTickRateRelation,
        powerCorrection,
            this.advPowerCorrection,
        encoderDataSource);
  }
}
