package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

import kotlin.Triple;

public interface ITapeMeasure extends Component, StatefulMechanism<Triple<Double, Double, Double>> {
  void adjustYaw(double amt);

  void adjustPitch(double amt);

  void setLengthRate(double amt);
}
