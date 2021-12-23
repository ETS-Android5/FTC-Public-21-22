package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.controller.Namable;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

public class RunToPositionTracker implements Namable {
  private final String name;
  private final TriFunction<Integer, Integer, Integer, Double> powerCurve;
  private int startingTicks;
  private int targetTicks;
  private double lastPower;

  public RunToPositionTracker(
      String name,
      TriFunction<Integer, Integer, Integer, Double> powerCurve,
      int startingTicks,
      int targetTicks,
      double lastPower) {
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

  public double getLastPower() {
    return lastPower;
  }

  public void setLastPower(double lastPower) {
    this.lastPower = lastPower;
  }

  public double getTargetPowerPercentage(int currentTicks) {
    double ret = powerCurve.apply(currentTicks, startingTicks, targetTicks);
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
