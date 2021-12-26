package org.firstinspires.ftc.teamcode.core.hardware.state;

public class MotorPositionData {
  private double ticks;
  private double velocity;

  public MotorPositionData(double ticks, double velocity) {
    addDataPoint(ticks, velocity);
  }

  public double getTicks() {
    return ticks;
  }

  public double getVelocity() {
    return velocity;
  }

  public void addDataPoint(double ticks, double velocity) {
    this.ticks = ticks;
    this.velocity = velocity;
  }
}
