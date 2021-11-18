package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Arrays;
import java.util.List;

public class AWDClassicDrivetrain implements IAWDClassicDrivetrain {
  private static final String FRONT_LEFT_MOTOR_NAME = "FRONT_LEFT_MOTOR";
  private static final String FRONT_RIGHT_MOTOR_NAME = "FRONT_RIGHT_MOTOR";
  private static final String REAR_LEFT_MOTOR_NAME = "REAR_LEFT_MOTOR";
  private static final String REAR_RIGHT_MOTOR_NAME = "REAR_RIGHT_MOTOR";

  public AWDClassicDrivetrain() {
    initialize();
  }

  @Hardware(name = FRONT_LEFT_MOTOR_NAME, direction = Direction.REVERSE)
  public DcMotor frontLeftMotor;

  @Hardware(name = FRONT_RIGHT_MOTOR_NAME)
  public DcMotor frontRightMotor;

  @Hardware(name = REAR_LEFT_MOTOR_NAME, direction = Direction.REVERSE)
  public DcMotor rearLeftMotor;

  @Hardware(name = REAR_RIGHT_MOTOR_NAME)
  public DcMotor rearRightMotor;

  private IMotorState frontLeftMotorState;
  private IMotorState frontRightMotorState;
  private IMotorState rearLeftMotorState;
  private IMotorState rearRightMotorState;

  private void initialize() {
    frontLeftMotorState = new MotorState(FRONT_LEFT_MOTOR_NAME, true);
    frontRightMotorState = new MotorState(FRONT_RIGHT_MOTOR_NAME, false);
    rearLeftMotorState = new MotorState(REAR_LEFT_MOTOR_NAME, true);
    rearRightMotorState = new MotorState(REAR_RIGHT_MOTOR_NAME, false);
  }

  @Override
  public void setAllPower(double power) {
    frontLeftMotorState = frontLeftMotorState.withPower(power);
    frontRightMotorState = frontRightMotorState.withPower(power);
    rearLeftMotorState = rearLeftMotorState.withPower(power);
    rearRightMotorState = rearRightMotorState.withPower(power);
  }

  @Override
  public void setLeftPower(double power) {
    frontLeftMotorState = frontLeftMotorState.withPower(power);
    rearLeftMotorState = rearLeftMotorState.withPower(power);
  }

  @Override
  public void setRightPower(double power) {
    frontRightMotorState = frontRightMotorState.withPower(power);
    rearRightMotorState = rearRightMotorState.withPower(power);
  }

  @Override
  public void setAllTarget(int ticks) {
    frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
    frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
    rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
    rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
  }

  @Override
  public void setLeftTarget(int ticks) {
    frontLeftMotorState = frontLeftMotorState.withTargetPosition(ticks);
    rearLeftMotorState = rearLeftMotorState.withTargetPosition(ticks);
  }

  @Override
  public void setRightTarget(int ticks) {
    frontRightMotorState = frontRightMotorState.withTargetPosition(ticks);
    rearRightMotorState = rearRightMotorState.withTargetPosition(ticks);
  }

  @Override
  public void reset() {
    initialize();
  }

  @Override
  public void setRunMode(RunMode runMode) {
    frontLeftMotorState = frontLeftMotorState.withRunMode(runMode);
    frontRightMotorState = frontRightMotorState.withRunMode(runMode);
    rearLeftMotorState = rearLeftMotorState.withRunMode(runMode);
    rearRightMotorState = rearRightMotorState.withRunMode(runMode);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Arrays.asList(
        frontLeftMotorState, frontRightMotorState, rearLeftMotorState, rearRightMotorState);
  }
}
