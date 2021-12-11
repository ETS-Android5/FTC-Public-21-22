package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface ISingleServoGripper extends Component, StatefulMechanism<SingleServoGripperState> {
  void open();

  void close();

  void toggle();
}
