package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

public class CallbackData {
  private final String motorName;
  private final int motorTargetPosition;
  private final int toleranceTicks;
  private final Runnable callback;

  public CallbackData(
      String motorName, int motorTargetPosition, int toleranceTicks, Runnable callback) {
    this.motorName = motorName;
    this.motorTargetPosition = motorTargetPosition;
    this.toleranceTicks = toleranceTicks;
    this.callback = callback;
  }

  public String getMotorName() {
    return motorName;
  }

  public int getMotorTargetPosition() {
    return motorTargetPosition;
  }

  public int getToleranceTicks() {
    return toleranceTicks;
  }

  public Runnable getCallback() {
    return callback;
  }
}
