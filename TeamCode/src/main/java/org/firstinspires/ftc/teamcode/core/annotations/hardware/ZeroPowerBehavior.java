package org.firstinspires.ftc.teamcode.core.annotations.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public enum ZeroPowerBehavior {
  BRAKE,
  COAST;

  public DcMotor.ZeroPowerBehavior primitiveConversion() {
    switch (this) {
      case BRAKE:
        return DcMotor.ZeroPowerBehavior.BRAKE;
      case COAST:
        return DcMotor.ZeroPowerBehavior.FLOAT;
    }
    return null;
  }
}
