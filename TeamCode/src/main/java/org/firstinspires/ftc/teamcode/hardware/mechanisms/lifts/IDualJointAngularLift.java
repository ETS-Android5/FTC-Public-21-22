package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface IDualJointAngularLift extends Component, StatefulMechanism<Pair<Integer, Double>> {
  void setArmOnePosition(int ticks);

  void setArmTwoPosition(double position);
}
