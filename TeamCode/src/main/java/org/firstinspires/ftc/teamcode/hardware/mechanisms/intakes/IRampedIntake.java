package org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes;

public interface IRampedIntake extends IIntake<RampedIntakeState> {
  void raise();

  void lower();
}
