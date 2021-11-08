package org.firstinspires.ftc.teamcode.core.annotations.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public enum RunMode {
  RUN_TO_POSITION,
  RUN_USING_ENCODER,
  RUN_WITHOUT_ENCODER,
  STOP_AND_RESET_ENCODER;

  public DcMotor.RunMode primitiveConversion() {
    switch (this) {
      case RUN_TO_POSITION:
      case RUN_USING_ENCODER:
        return DcMotor.RunMode.RUN_USING_ENCODER;
      case RUN_WITHOUT_ENCODER:
        return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
      case STOP_AND_RESET_ENCODER:
        return DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    }
    return null;
  }
}
