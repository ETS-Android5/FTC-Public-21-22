package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface IFourHeightLift extends Component, StatefulMechanism<FourHeightLiftState> {
  void goToHeight0();

  void goToHeight1();

  void goToHeight2();

  void goToHeight3();
}
