package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;

public class ServoState extends IServoState {
  private final Direction direction;
  private final double position;
  private final boolean isContinuousRotation;

  public ServoState(
      String name, Direction direction, double position, boolean isContinuousRotation) {
    super(name);
    this.direction = direction;
    this.position = position;
    this.isContinuousRotation = isContinuousRotation;
  }

  @Override
  public Direction getDirection() {
    return direction;
  }

  @Override
  public IServoState withDirection(Direction direction) {
    return new ServoState(this.name, direction, this.position, this.isContinuousRotation);
  }

  @Override
  public double getPosition() {
    return position;
  }

  @Override
  public IServoState withPosition(double position) {
    return new ServoState(this.name, this.direction, position, this.isContinuousRotation);
  }

  @Override
  public boolean isContinuousRotation() {
    return isContinuousRotation;
  }

  @Override
  public IServoState withContinuousRotation(boolean isContinuousRotation) {
    return new ServoState(this.name, this.direction, this.position, isContinuousRotation);
  }
}
