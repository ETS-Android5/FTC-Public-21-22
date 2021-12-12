package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class Turret implements ITurret {
  private static final String TURRET_MOTOR_NAME = "TURRET_MOTOR";
  private static final int DEGREES_FRONT = 0;
  private static final int DEGREES_RIGHT = 90;
  private static final int DEGREES_BACK = 180;
  private static final double DEGREES_TO_TICKS = 1 / 1.25; // Rev core hex is 288 ticks / rotation

  @Hardware(
      name = TURRET_MOTOR_NAME,
      runMode = RunMode.RUN_TO_POSITION,
      direction = Direction.REVERSE)
  public DcMotor turretMotor;

  private IMotorState turretMotorState;

  public Turret() {
    initialize();
  }

  private void initialize() {
    turretMotorState =
        new MotorState(TURRET_MOTOR_NAME, Direction.REVERSE)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(0)
            .withPowerCurve(PowerCurves.RUN_TO_POSITION_TENTH_POWER);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Collections.singletonList(turretMotorState.duplicate());
  }

  @Override
  @Observable(key = "TURRET")
  public Integer getState() {
    return (int) Math.round(turretMotorState.getTargetPosition() / DEGREES_TO_TICKS);
  }

  @Override
  public void turnToDegrees(int degrees) {
    int safeDegrees = degrees % 360;
    if (safeDegrees > 180 || safeDegrees < -180) {
      return;
    }
    synchronized (this) {
      turretMotorState =
          turretMotorState.withTargetPosition((int) Math.round((degrees % 360) * DEGREES_TO_TICKS));
    }
  }

  @Override
  public synchronized void turnToFront() {
    turnToDegrees(DEGREES_FRONT);
  }

  @Override
  public synchronized void turnToLeft() {
    turnToDegrees(-DEGREES_RIGHT);
  }

  @Override
  public synchronized void turnToRight() {
    turnToDegrees(DEGREES_RIGHT);
  }

  @Override
  public synchronized void turnCWToBack() {
    turnToDegrees(DEGREES_BACK);
  }

  @Override
  public synchronized void turnCCWToBack() {
    turnToDegrees(-DEGREES_BACK);
  }

  @Override
  public synchronized void turnToBackNearest() {
    if (turretMotorState.getTargetPosition() < 0) {
      turnCCWToBack();
    } else {
      turnCWToBack();
    }
  }
}
