package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;

public class DualJointAngularLift implements IDualJointAngularLift {
  public static final String LIFT_JOINT_ONE_MOTOR_NAME = "LIFT_JOINT_ONE_MOTOR";
  public static final String LIFT_JOINT_TWO_SERVO_NAME = "LIFT_JOINT_TWO_SERVO";
  public static final int DEFAULT_ADJUSTMENT_THRESHOLD = 50;

  @Hardware(
      name = LIFT_JOINT_ONE_MOTOR_NAME,
      runMode = RunMode.RUN_TO_POSITION,
      direction = Direction.REVERSE)
  @SuppressWarnings("unused")
  public DcMotorEx liftJointOneMotor;

  @Hardware(name = LIFT_JOINT_TWO_SERVO_NAME)
  @SuppressWarnings("unused")
  public Servo liftJointTwoServo;

  private IMotorState liftJointOneMotorState;
  private IServoState liftJointTwoServoState;

  public DualJointAngularLift() {
    initialize();
  }

  private void initialize() {
    liftJointOneMotorState =
        new MotorState(LIFT_JOINT_ONE_MOTOR_NAME, Direction.REVERSE)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(0)
            .withPowerCurve(PowerCurves.generatePowerCurve(1, 1))
            .withAdjustmentThreshold(DEFAULT_ADJUSTMENT_THRESHOLD)
            .withAdjustmentCurve(PowerCurves.generatePowerCurve(0.05, 0.25))
            .withAdjustmentPowerCorrectionCurve(
                (Double currentPower, Double idealPower, Double percentProgress) -> {
                  if (currentPower.equals(idealPower)) return currentPower;
                  double diff = Math.abs(currentPower - idealPower);
                  double adjustment =
                      Math.min(Math.pow(Math.abs(1 - percentProgress), 2), 1)
                          * .12
                          * Math.pow(diff, 1.0 / 2);
                  double ret = 0;
                  if (currentPower > idealPower) ret = currentPower - adjustment;
                  if (currentPower < idealPower) ret = currentPower + adjustment;
                  ret = Math.round(ret * 100) / 100.0;
                  double clippedRange = ret > .85 ? .85 : ret < -.85 ? -.85 : ret;
                  clippedRange = clippedRange == 0 ? Double.MIN_VALUE : clippedRange;
                  return clippedRange;
                })
            .withPowerAndTickRateRelation((power) -> power * 2800)
            .withPowerCorrection(
                (Double currentPower, Double idealPower, Double percentProgress) -> {
                  if (currentPower.equals(idealPower)) return currentPower;
                  double diff = Math.abs(currentPower - idealPower);

                  double adjustment =
                      Math.min(Math.pow(Math.abs(1 - percentProgress), 1), 1)
                          * 2
                          * Math.pow(diff, 1.0 / 2);
                  double ret = 0;
                  if (currentPower > idealPower) ret = currentPower - adjustment;
                  if (currentPower < idealPower) ret = currentPower + adjustment;
                  ret = Math.round(ret * 100) / 100.0;
                  double clippedRange = ret > 1 ? 1 : ret < -1 ? -1 : ret;
                  clippedRange = clippedRange == 0 ? Double.MIN_VALUE : clippedRange;
                  return clippedRange;
                });
    liftJointTwoServoState = new ServoState(LIFT_JOINT_TWO_SERVO_NAME, Direction.FORWARD, 0.55);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Arrays.asList(liftJointOneMotorState.duplicate(), liftJointTwoServoState);
  }

  @Override
  @Observable(key = "DUAL_JOINT_ANGULAR_LIFT")
  public Pair<Integer, Double> getState() {
    return new Pair<>(
        liftJointOneMotorState.getTargetPosition(), liftJointTwoServoState.getPosition());
  }

  @Override
  public synchronized void setArmOnePosition(int ticks) {
    liftJointOneMotorState = liftJointOneMotorState.withTargetPosition(ticks);
  }

  @Override
  public synchronized void setArmTwoPosition(double position) {
    liftJointTwoServoState = liftJointTwoServoState.withPosition(position);
  }

  @Override
  public synchronized void setAdjustmentThreshold(int ticks) {
    liftJointOneMotorState = liftJointOneMotorState.withAdjustmentThreshold(ticks);
  }
}
