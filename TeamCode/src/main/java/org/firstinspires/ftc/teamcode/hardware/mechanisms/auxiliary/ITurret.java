package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface ITurret extends Component, StatefulMechanism<Double> {
  void turnToDegrees(double degrees);

  void turnToPosition(double position);

  void turnToFront();

  void turnToLeft();

  void turnToRight();

  void turnCWToBack();

  void turnCCWToBack();

  void turnToBackNearest();
}
