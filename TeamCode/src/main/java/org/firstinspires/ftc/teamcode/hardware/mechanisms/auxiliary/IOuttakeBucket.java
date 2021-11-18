package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface IOuttakeBucket extends Component, StatefulMechanism<OuttakeBucketState> {
  void dump();

  void carry();
}
