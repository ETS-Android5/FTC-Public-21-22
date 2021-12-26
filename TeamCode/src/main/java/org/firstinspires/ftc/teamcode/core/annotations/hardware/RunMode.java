package org.firstinspires.ftc.teamcode.core.annotations.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public enum RunMode {
  RUN_TO_POSITION,
  RUN_USING_ENCODER,
  RUN_WITHOUT_ENCODER,
  STOP_AND_RESET_ENCODER;

  public DcMotorEx.RunMode primitiveConversion() {
    switch (this) {
      case RUN_TO_POSITION:
      case RUN_USING_ENCODER:
        return DcMotorEx.RunMode.RUN_USING_ENCODER;
      case RUN_WITHOUT_ENCODER:
        return DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
      case STOP_AND_RESET_ENCODER:
        return DcMotorEx.RunMode.STOP_AND_RESET_ENCODER;
    }
    return null;
  }
}
