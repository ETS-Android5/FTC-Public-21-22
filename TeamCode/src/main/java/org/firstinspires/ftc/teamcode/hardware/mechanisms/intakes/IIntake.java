package org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface IIntake<T> extends Component, StatefulMechanism<T> {
  void toggleIntaking();

  void toggleOuttaking();

  void beginIntaking();

  void beginOuttaking();

  void beginOuttakingSlowly();

  void stop();
}
