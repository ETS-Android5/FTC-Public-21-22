package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.controller.Namable;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;

public class RunToPositionTracker implements Namable {
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

  public void mutateTo(int startingTicks, int targetTicks) {
    this.startingTicks = startingTicks;
    this.targetTicks = targetTicks;
  }

  private final String name;
  private final TriFunction<Integer, Integer, Integer, Double> powerCurve;
  private int startingTicks;
  private int targetTicks;
  private double currentPower;
  private double nextPower;

  public double getTargetPowerPercentage(int currentTicks) {
    double ret = powerCurve.apply(currentTicks, startingTicks, targetTicks);
    ret = ret < -1 ? -1 : ret > 1 ? 1 : ret;
    currentPower = nextPower;
    nextPower = ret;
    return ret;
  }

  public boolean shouldUpdatePower() {
    return nextPower != currentPower;
  }

  @Override
  public String getName() {
    return name;
  }
}
