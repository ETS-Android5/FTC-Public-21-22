package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

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
  public abstract TriFunction<Integer, Integer, Integer, Double> getPowerCurve();

  public abstract IMotorState withPowerCurve(
      TriFunction<Integer, Integer, Integer, Double> powerCurve);

  @Observable(key = "MaxAcceleration")
  public abstract double getMaxAcceleration();

  public abstract IMotorState withMaxAcceleration(double maxAcceleration);
}
