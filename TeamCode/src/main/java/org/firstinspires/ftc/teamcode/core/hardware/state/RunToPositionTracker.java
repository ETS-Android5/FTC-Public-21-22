package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.Namable;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

public class RunToPositionTracker implements Namable {
  private final String name;
  private final TriFunction<Double, Double, Double, Double> powerCurve;
  private int startingTicks;
  private int targetTicks;
  private double lastPower;

  public RunToPositionTracker(
      String name,
      TriFunction<Double, Double, Double, Double> powerCurve,
      int startingTicks,
      int targetTicks) {
    this.name = name;
    this.powerCurve = powerCurve;
    this.startingTicks = startingTicks;
    this.targetTicks = targetTicks;
  }

  public int getTargetTicks() {
    return targetTicks;
  }

  public void mutateTo(int startingTicks, int targetTicks) {
    this.startingTicks = startingTicks;
    this.targetTicks = targetTicks;
  }

  public void setLastPower(double lastPower) {
    this.lastPower = lastPower;
  }

  public double getTargetPowerPercentage(int currentTicks) {
    double ret =
        powerCurve.apply((double) currentTicks, (double) startingTicks, (double) targetTicks);
    return ret < -1 ? -1 : ret > 1 ? 1 : ret;
  }

  public boolean shouldUpdatePower(int currentTicks) {
    return lastPower != getTargetPowerPercentage(currentTicks);
  }

  @Override
  public String getName() {
    return name;
  }
}
