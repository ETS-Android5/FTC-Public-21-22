package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;

public class ServoState extends IServoState {
  private final Direction direction;
  private final double position;

  public ServoState(String name, Direction direction, double position) {
    super(name);
    this.direction = direction;
    this.position = position;
  }

  @Override
  public Direction getDirection() {
    return direction;
  }

  @Override
  public IServoState withDirection(Direction direction) {
    return new ServoState(this.name, direction, this.position);
  }

  @Override
  public double getPosition() {
    return position;
  }

  @Override
  public IServoState withPosition(double position) {
    return new ServoState(this.name, this.direction, position);
  }
}
