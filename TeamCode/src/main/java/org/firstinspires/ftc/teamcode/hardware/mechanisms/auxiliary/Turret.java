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
  public static final int DEFAULT_ADJUSTMENT_THRESHOLD = 40;
  public static final double DEGREES_FRONT = 0;
  public static final int DEGREES_DIAGONAL = 135;
  public static final int DEGREES_LEFT = -81;
  public static final int DEGREES_RIGHT = 84;
  public static final int DEGREES_BACK = 180;
  public static final double DEGREES_TO_TICKS = 7.7394444444;
  public static final int TICKS_FRONT = 0;
  public static final int TICKS_RIGHT = (int) Math.round(DEGREES_RIGHT * DEGREES_TO_TICKS);
  public static final int TICKS_LEFT = (int) Math.round(DEGREES_LEFT * DEGREES_TO_TICKS);
  public static final int TICKS_DIAGONAL_RIGHT =
      (int) Math.round(DEGREES_DIAGONAL * DEGREES_TO_TICKS);
  public static final int TICKS_DIAGONAL_LEFT =
      (int) Math.round(-DEGREES_DIAGONAL * DEGREES_TO_TICKS);
  public static final int TICKS_CW_BACK = (int) Math.round(DEGREES_BACK * DEGREES_TO_TICKS);
  public static final int TICKS_CCW_BACK = (int) Math.round(-DEGREES_BACK * DEGREES_TO_TICKS);

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
            .withPowerCurve(PowerCurves.generatePowerCurve(1, 2))
            .withAdjustmentThreshold(DEFAULT_ADJUSTMENT_THRESHOLD)
            .withAdjustmentCurve(PowerCurves.generatePowerCurve(0.05, 0.25))
            .withAdjustmentPowerCorrectionCurve(
                (Double currentPower, Double idealPower, Double percentProgress) -> {
                  if (currentPower.equals(idealPower)) return currentPower;
                  double diff = Math.abs(currentPower - idealPower);
                  double adjustment =
                      Math.min(Math.pow(Math.abs(1 - percentProgress), 4), 1)
                          * Math.pow(diff, 1.0 / 1.75)
                          * .1;
                  double ret = 0;
                  if (currentPower > idealPower) ret = idealPower - adjustment;
                  if (currentPower < idealPower) ret = idealPower + adjustment;
                  ret = Math.round(ret * 100) / 100.0;
                  double clippedRange = ret > .25 ? .25 : ret < -.25 ? -.25 : ret;
                  clippedRange = clippedRange == 0 ? Double.MIN_VALUE : clippedRange;
                  return clippedRange;
                })
            .withPowerAndTickRateRelation((power) -> power * 2786.2) // Ticks / second at 100% power
            .withPowerCorrection(
                (Double currentPower, Double idealPower, Double percentProgress) -> {
                  if (currentPower.equals(idealPower)) return currentPower;
                  // If the turret is moving faster than full speed, why slow it down?
                  if ((idealPower == 1 && currentPower >= 1)
                      || (idealPower == -1 && currentPower <= -1)) {
                    return idealPower;
                  }
                  double diff = Math.abs(currentPower - idealPower);

                  double adjustment =
                      Math.min(Math.pow(Math.abs(1 - percentProgress), 4), 1)
                          * Math.pow(diff, 1.0 / 1.75);
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
  public void turnToFront() {
    turnToDegrees(DEGREES_FRONT);
  }

  @Override
  public void turnToLeft() {
    turnToDegrees(DEGREES_LEFT);
  }

  @Override
  public void turnToRight() {
    turnToDegrees(DEGREES_RIGHT);
  }

  @Override
  public void turnToDiagonalLeft() {
    turnToDegrees(-DEGREES_DIAGONAL);
  }

  @Override
  public void turnToDiagonalRight() {
    turnToDegrees(DEGREES_DIAGONAL);
  }

  @Override
  public void turnCWToBack() {
    turnToDegrees(DEGREES_BACK);
  }

  @Override
  public void turnCCWToBack() {
    turnToDegrees(-DEGREES_BACK);
  }

  @Override
  public void turnToBackNearest() {
    if (turretMotorState.getTargetPosition() < 0) {
      turnCCWToBack();
    } else {
      turnCWToBack();
    }
  }

  @Override
  public synchronized void setAdjustmentThreshold(int ticks) {
    turretMotorState = turretMotorState.withAdjustmentThreshold(ticks);
  }
}
