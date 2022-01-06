package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotorEx;

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
  public static final String TURRET_MOTOR_NAME = "TURRET_MOTOR";
  public static final int DEGREES_FRONT = 0;
  public static final int DEGREES_RIGHT = 90;
  public static final int DEGREES_BACK = 180;
  public static final double DEGREES_TO_TICKS = 2.133333333333333333333333333;
  public static final int TICKS_FRONT = 0;
  public static final int TICKS_RIGHT = (int) ((DEGREES_RIGHT * DEGREES_TO_TICKS) + 0.5);
  public static final int TICKS_LEFT = (int) ((-DEGREES_RIGHT * DEGREES_TO_TICKS) - 0.5);
  public static final int TICKS_CW_BACK = (int) ((DEGREES_BACK * DEGREES_TO_TICKS) + 0.5);
  public static final int TICKS_CCW_BACK = (int) ((-DEGREES_BACK * DEGREES_TO_TICKS) - 0.5);

  @Hardware(name = TURRET_MOTOR_NAME, runMode = RunMode.RUN_TO_POSITION)
  @SuppressWarnings("unused")
  public DcMotorEx turretMotor;

  private IMotorState turretMotorState;

  public Turret() {
    initialize();
  }

  private void initialize() {
    turretMotorState =
        new MotorState(TURRET_MOTOR_NAME, Direction.FORWARD)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(0)
            .withPowerCurve(PowerCurves.generatePowerCurve(1, .9))
            .withPowerAndTickRateRelation((power) -> power * 600) // 5900 tps at 100% power
            .withPowerCorrection(
                (Double currentPower,
                    Double idealPower,
                    Integer currentTicks,
                    Integer targetTicks) -> {
                  if (currentPower.equals(idealPower)) return currentPower;
                  // If the turret is moving faster than full speed, why slow it down?
                  if ((idealPower == 1 && currentPower >= 1)
                      || (idealPower == -1 && currentPower <= -1)) {
                    return idealPower;
                  }
                  double diff = Math.abs(currentPower - idealPower);
                  double adjustment =
                      Math.min(Math.pow(Math.abs(currentTicks - targetTicks) / 2.0, 2), 1)
                          * Math.pow(diff, 1.0 / 3.5);

                  double ret = 0;
                  if (currentPower > idealPower) ret = idealPower - adjustment;
                  if (currentPower < idealPower) ret = idealPower + adjustment;
                  ret = Math.round(ret * 100) / 100.0;
                  double clippedRange = ret > 1 ? 1 : ret < -1 ? -1 : ret;
                  clippedRange = clippedRange == 0 ? Double.MIN_VALUE : clippedRange;
                  return clippedRange;
                });
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
  public Double getState() {
    return turretMotorState.getTargetPosition() / DEGREES_TO_TICKS;
  }

  @Override
  public void turnToDegrees(double degrees) {
    if (degrees > 180 || degrees < -180) {
      return;
    }
    synchronized (this) {
      turretMotorState =
          turretMotorState.withTargetPosition((int) Math.round(degrees * DEGREES_TO_TICKS));
    }
  }

  @Override
  public void turnToPosition(double position) {
    if (position > TICKS_CW_BACK || position < TICKS_CCW_BACK) {
      return;
    }
    synchronized (this) {
      turretMotorState = turretMotorState.withTargetPosition((int) Math.round(position));
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
