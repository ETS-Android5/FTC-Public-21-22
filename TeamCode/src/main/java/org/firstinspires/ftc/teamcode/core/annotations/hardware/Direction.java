package org.firstinspires.ftc.teamcode.core.annotations.hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public enum Direction {
  FORWARD,
  REVERSE;

  public DcMotorSimple.Direction primitiveConversion() {
    switch (this) {
      case FORWARD:
        return DcMotorSimple.Direction.FORWARD;
      case REVERSE:
        return DcMotorSimple.Direction.REVERSE;
    }
    return null;
  }

  public Servo.Direction primitiveServoConversion() {
    switch (this) {
      case FORWARD:
        return Servo.Direction.FORWARD;
      case REVERSE:
        return Servo.Direction.REVERSE;
    }
    return null;
  }
}
