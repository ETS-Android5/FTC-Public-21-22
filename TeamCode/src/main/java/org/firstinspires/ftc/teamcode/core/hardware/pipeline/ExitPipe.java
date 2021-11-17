package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

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
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    r.getNextMotorStates().forEach((m) -> processMotor(m, hardware));
    r.getNextServoStates().forEach((s) -> processServo(s, hardware));
    return super.process(hardware, r);
  }

  private void processMotor(IMotorState nextState, Map<String, Object> hardware) {
    if (nextState != null) {
      IMotorState currentState = State.currentStateOf(nextState.getName());
      DcMotor motorObj = (DcMotor) hardware.get(nextState.getName());
      if (motorObj != null) {
        if (currentState == null || currentState.getDirection() != nextState.getDirection()) {
          motorObj.setDirection(nextState.getDirection().primitiveConversion());
        }
        if (currentState == null || currentState.getRunMode() != nextState.getRunMode()) {
          motorObj.setMode(nextState.getRunMode().primitiveConversion());
        }
        if (currentState == null
            || currentState.getZeroPowerBehavior() != nextState.getZeroPowerBehavior()) {
          motorObj.setZeroPowerBehavior(nextState.getZeroPowerBehavior().primitiveConversion());
        }
        if (currentState == null || currentState.getPower() != nextState.getPower()) {
          motorObj.setPower(nextState.getPower());
        }
        if (currentState == null
            || currentState.getTargetPosition() != nextState.getTargetPosition()) {
          motorObj.setTargetPosition(nextState.getTargetPosition());
        }
        nextState.makeCurrent();
      }
    }
  }

  private void processServo(IServoState nextState, Map<String, Object> hardware) {
    if (nextState != null) {
      IServoState currentState = State.currentStateOf(nextState.getName());
      Servo servoObj = (Servo) hardware.get(nextState.getName());
      if (servoObj != null) {
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
}
