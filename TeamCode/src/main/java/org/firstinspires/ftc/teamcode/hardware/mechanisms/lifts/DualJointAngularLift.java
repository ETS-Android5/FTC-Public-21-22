package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
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
  public static final double LIFT_JOINT_TWO_INTAKE_POSITION = 0.554;
  public static final double TICKS_IN_90_DEGREES = 896;
  public final int offset;
  public final int degrees90Position;

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

  public DualJointAngularLift(int offset) {
    this.offset = offset;
    this.degrees90Position = offset - 90;
    initialize();
  }

  private void initialize() {
    liftJointOneMotorState =
        new MotorState(LIFT_JOINT_ONE_MOTOR_NAME, Direction.REVERSE)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(0)
            .withPowerCurve(
                PowerCurves.generatePowerCurve(1, 2)
                    .andThen(
                        (Double powerResult) -> {
                          double anglePower =
                              Math.cos(
                                  (Math.abs(
                                              MotorTrackerPipe.getInstance()
                                                      .getPositionOf(LIFT_JOINT_ONE_MOTOR_NAME)
                                                  - degrees90Position)
                                          / TICKS_IN_90_DEGREES)
                                      * Math.PI
                                      * 0.5);
                          anglePower += 0.06;
                          return powerResult * anglePower * (powerResult < 0 ? 0.333333 : 1);
                        }))
            .withPowerAndTickRateRelation((power) -> power * 2800)
            .withAdvPowerCorrection(
                (Double currentPower,
                    Double idealPower,
                    Double currentTicks,
                    Double targetTicks) -> {
                  if (currentPower.equals(idealPower)) return currentPower;

                  double diff = Math.abs(currentTicks - targetTicks);
                  double pow = Math.pow(Math.abs(1 - (diff / (TICKS_IN_90_DEGREES * 0.01))), 3);
                  double adjustment = Math.min(pow, 1) * .1;
                  double ret = 0;
                  if (currentPower > idealPower) {
                    ret = currentPower - adjustment;
                    if (ret > 0) ret = -0.01;
                  } else if (currentPower < idealPower) {
                    ret = currentPower + adjustment;
                    if (ret < 0) ret = 0.01;
                  }
                  ret = Math.round(ret * 1000) / 1000.0;
                  double clippedRange = ret > 1 ? 1 : ret < -1 ? -1 : ret;
                  clippedRange = clippedRange == 0 ? Double.MIN_VALUE : clippedRange;
                  return clippedRange;
                });
    liftJointTwoServoState =
        new ServoState(
            LIFT_JOINT_TWO_SERVO_NAME, Direction.FORWARD, LIFT_JOINT_TWO_INTAKE_POSITION, false);
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
    liftJointOneMotorState = liftJointOneMotorState.withTargetPosition(offset + ticks);
  }

  @Override
  public synchronized void setArmTwoPosition(double position) {
    liftJointTwoServoState = liftJointTwoServoState.withPosition(position);
  }

  @Override
  public int getArmOneOffset() {
    return offset;
  }

  @Override
  public synchronized void setAdjustmentThreshold(int ticks) {
    liftJointOneMotorState = liftJointOneMotorState.withAdjustmentThreshold(ticks);
  }
}
