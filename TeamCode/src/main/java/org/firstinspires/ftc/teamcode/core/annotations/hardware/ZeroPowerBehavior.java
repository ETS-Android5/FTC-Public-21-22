package org.firstinspires.ftc.teamcode.core.annotations.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public enum ZeroPowerBehavior {
  BRAKE,
  COAST;

  public DcMotorEx.ZeroPowerBehavior primitiveConversion() {
    switch (this) {
      case BRAKE:
        return DcMotorEx.ZeroPowerBehavior.BRAKE;
      case COAST:
        return DcMotorEx.ZeroPowerBehavior.FLOAT;
    }
    return null;
  }
}
