package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
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
  private static final String LIFT_JOINT_ONE_MOTOR_NAME = "LIFT_JOINT_ONE_MOTOR";
  private static final String LIFT_JOINT_TWO_SERVO_NAME = "LIFT_JOINT_TWO_SERVO";

  @Hardware(name = LIFT_JOINT_ONE_MOTOR_NAME, runMode = RunMode.RUN_TO_POSITION)
  public DcMotor liftJointOneMotor;

  @Hardware(name = LIFT_JOINT_TWO_SERVO_NAME)
  public Servo liftJointTwoServo;

  private IMotorState liftJointOneMotorState;
  private IServoState liftJointTwoServoState;

  public DualJointAngularLift() {
    initialize();
  }

  private void initialize() {
    liftJointOneMotorState =
        new MotorState(LIFT_JOINT_ONE_MOTOR_NAME, Direction.FORWARD)
            .withRunMode(RunMode.RUN_TO_POSITION)
            .withTargetPosition(0)
            .withPowerCurve(PowerCurves.RUN_TO_POSITION_ANGULAR_LIFT_CURVE);
    liftJointTwoServoState = new ServoState(LIFT_JOINT_TWO_SERVO_NAME, Direction.FORWARD, 0);
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
}
