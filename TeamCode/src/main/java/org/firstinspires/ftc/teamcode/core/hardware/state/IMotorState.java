package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.core.fn.QuadFunction;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

import java.util.function.Function;

public abstract class IMotorState extends State implements Duplicatable<IMotorState> {
  IMotorState(String name) {
    super(name);
  }

  @Observable(key = "Direction")
  public abstract Direction getDirection();

  public abstract IMotorState withDirection(Direction direction);

  @Observable(key = "RunMode")
  public abstract RunMode getRunMode();

  public abstract IMotorState withRunMode(RunMode runMode);

  @Observable(key = "ZeroPowerBehavior")
  public abstract ZeroPowerBehavior getZeroPowerBehavior();

  public abstract IMotorState withZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);

  @Observable(key = "Power")
  public abstract double getPower();

  public abstract IMotorState withPower(double power);

  @Observable(key = "TargetPosition")
  public abstract int getTargetPosition();

  public abstract IMotorState withTargetPosition(int targetPosition);

  @Observable(key = "PowerCurve")
  public abstract TriFunction<Double, Double, Double, Double> getPowerCurve();

  public abstract IMotorState withPowerCurve(
      TriFunction<Double, Double, Double, Double> powerCurve);

  @Observable(key = "AdjustmentThreshold")
  public abstract int getAdjustmentThreshold();

  public abstract IMotorState withAdjustmentThreshold(int adjustmentThreshold);

  @Observable(key = "AdjustmentCurve")
  public abstract TriFunction<Double, Double, Double, Double> getAdjustmentCurve();

  public abstract IMotorState withAdjustmentCurve(
      TriFunction<Double, Double, Double, Double> adjustmentCurve);

  @Observable(key = "AdjustmentPowerCorrectionCurve")
  public abstract TriFunction<Double, Double, Double, Double> getAdjustmentPowerCorrectionCurve();

  public abstract IMotorState withAdjustmentPowerCorrectionCurve(
      TriFunction<Double, Double, Double, Double> adjustmentPowerCorrectionCurve);

  @Observable(key = "PowerAndTickRateRelation")
  public abstract Function<Double, Double> getPowerAndTickRateRelation();

  public abstract IMotorState withPowerAndTickRateRelation(
      Function<Double, Double> powerAndTickRateRelation);

  @Observable(key = "PowerCorrection")
  public abstract TriFunction<Double, Double, Double, Double> getPowerCorrection();

  public abstract IMotorState withPowerCorrection(
      TriFunction<Double, Double, Double, Double> correctionAggressionCurve);

  @Observable(key = "AdvPowerCorrection")
  public abstract QuadFunction<Double, Double, Double, Double, Double> getAdvPowerCorrection();

  public abstract IMotorState withAdvPowerCorrection(QuadFunction<Double, Double, Double, Double, Double> advPowerCorrection);

  @Observable(key = "EncoderDataSource")
  public abstract String getEncoderDataSource();

  public abstract IMotorState withEncoderDataSource(String encoderDataSource);
}
