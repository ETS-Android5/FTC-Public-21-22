package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class ExitPipe extends HardwarePipeline {
  public ExitPipe(String name) {
    super(name);
  }

  @Override
  public Component process(Map<String, Object> hardware, Component c) {
    List<IMotorState> motors =
        c.getNextState().stream()
            .filter((s) -> s instanceof IMotorState)
            .map((s) -> (IMotorState) s)
            .collect(Collectors.toList());
    List<IServoState> servos =
        c.getNextState().stream()
            .filter((s) -> s instanceof IServoState)
            .map((s) -> (IServoState) s)
            .collect(Collectors.toList());
    motors.forEach((m) -> processMotor(m.getName(), hardware));
    servos.forEach((s) -> processServo(s.getName(), hardware));
    return super.process(hardware, c);
  }

  private void processMotor(String motor, Map<String, Object> hardware) {
    IMotorState currentState = State.currentStateOf(motor);
    IMotorState nextState = State.nextStateOf(motor);
    DcMotor motorObj = (DcMotor) hardware.get(motor);
    if (motorObj != null && nextState != null) {
      if (currentState == null || currentState.getDirection() != nextState.getDirection()) {
        motorObj.setDirection(nextState.getDirection().primitiveConversion());
      }
      if (currentState == null || currentState.getRunMode() != nextState.getRunMode()) {
        motorObj.setMode(nextState.getRunMode().primitiveConversion());
      }
      if (currentState == null || currentState.getZeroPowerBehavior() != nextState.getZeroPowerBehavior()) {
        motorObj.setZeroPowerBehavior(nextState.getZeroPowerBehavior().primitiveConversion());
      }
      if (currentState == null || currentState.getPower() != nextState.getPower()) {
        motorObj.setPower(nextState.getPower());
      }
      if (currentState == null || currentState.getTargetPosition() != nextState.getTargetPosition()) {
        motorObj.setTargetPosition(nextState.getTargetPosition());
      }
      nextState.makeCurrent();
    }
  }

  private void processServo(String servo, Map<String, Object> hardware) {
    IServoState currentState = State.currentStateOf(servo);
    IServoState nextState = State.nextStateOf(servo);
    Servo servoObj = (Servo) hardware.get(servo);
    if (servoObj != null && nextState != null) {
      if (currentState == null || currentState.getDirection() != nextState.getDirection()) {
        servoObj.setDirection(nextState.getDirection().primitiveServoConversion());
      }
      if (currentState == null || currentState.getPosition() != nextState.getPosition()) {
        servoObj.setPosition(nextState.getPosition());
      }
      nextState.makeCurrent();
    }
  }
}
