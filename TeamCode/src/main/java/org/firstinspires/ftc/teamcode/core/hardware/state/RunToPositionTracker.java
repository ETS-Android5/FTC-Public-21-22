package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.Namable;

public class RunToPositionTracker implements Namable {
  private final String name;
  private int startingTicks;
  private int targetTicks;
  private double lastPower;

  public RunToPositionTracker(String name, int startingTicks, int targetTicks) {
    this.name = name;
    this.startingTicks = startingTicks;
    this.targetTicks = targetTicks;
  }

  public int getStartingTicks() {
    return startingTicks;
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
    double ret;
    IMotorState motorState = State.nextStateOf(name);
    if (motorState == null) motorState = State.currentStateOf(name);
    if (motorState == null) return 0;
    if (motorState.getAdjustmentCurve() != null
        && Math.abs(startingTicks - targetTicks) < motorState.getAdjustmentThreshold()) {
      ret =
          motorState
              .getAdjustmentCurve()
              .apply((double) currentTicks, (double) startingTicks, (double) targetTicks);
    } else {
      ret =
          motorState
              .getPowerCurve()
              .apply((double) currentTicks, (double) startingTicks, (double) targetTicks);
    }
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
