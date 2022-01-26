package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;

public abstract class IServoState extends State {
  IServoState(String name) {
    super(name);
  }

  @Observable(key = "Direction")
  public abstract Direction getDirection();

  public abstract IServoState withDirection(Direction direction);

  @Observable(key = "Position")
  public abstract double getPosition();

  public abstract IServoState withPosition(double position);
}
