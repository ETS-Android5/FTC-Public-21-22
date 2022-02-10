package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.ManuallyAdjustable;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface ITurret extends Component, ManuallyAdjustable, StatefulMechanism<Double> {
  void turnToDegrees(double degrees);

  void turnToPosition(double position);

  void turnToFront();

  void turnToLeft();

  void turnToRight();

  void turnToDiagonalLeft();

  void turnToDiagonalRight();

  void turnCWToBack();

  void turnCCWToBack();

  void turnToBackNearest();
}
