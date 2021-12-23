package org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes;

public enum RampedIntakeState {
  LOWERED_AND_STOPPED,
  LOWERED_AND_INTAKING,
  LOWERED_AND_OUTTAKING,
  LOWERED_AND_OUTTAKING_SLOWLY,
  RAISED_AND_STOPPED,
  RAISED_AND_INTAKING,
  RAISED_AND_OUTTAKING,
  RAISED_AND_OUTTAKING_SLOWLY;

  public boolean isLowered() {
    return this == LOWERED_AND_STOPPED
        || this == LOWERED_AND_INTAKING
        || this == LOWERED_AND_OUTTAKING
        || this == LOWERED_AND_OUTTAKING_SLOWLY;
  }
}
