package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Map;

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

  @SuppressWarnings("all")
  private void processMotor(IMotorState nextState, Map<String, Object> hardware) {
    IMotorState currentState = State.currentStateOf(nextState.getName());
    DcMotor motorObj = (DcMotor) hardware.get(nextState.getName());
    boolean noState = currentState == null;
    if (noState || currentState.getDirection() != nextState.getDirection()) {
      motorObj.setDirection(nextState.getDirection().primitiveConversion());
    }
    if (noState || currentState.getRunMode() != nextState.getRunMode()) {
      motorObj.setMode(nextState.getRunMode().primitiveConversion());
    }
    if (noState || currentState.getZeroPowerBehavior() != nextState.getZeroPowerBehavior()) {
      motorObj.setZeroPowerBehavior(nextState.getZeroPowerBehavior().primitiveConversion());
    }
    if (noState || currentState.getPower() != nextState.getPower()) {
      motorObj.setPower(nextState.getPower());
    }
    if (noState || currentState.getTargetPosition() != nextState.getTargetPosition()) {
      motorObj.setTargetPosition(nextState.getTargetPosition());
    }
    nextState.makeCurrent();
  }

  @SuppressWarnings("all")
  private void processServo(IServoState nextState, Map<String, Object> hardware) {
    IServoState currentState = State.currentStateOf(nextState.getName());
    Servo servoObj = (Servo) hardware.get(nextState.getName());
    boolean noState = currentState == null;
    if (noState || currentState.getDirection() != nextState.getDirection()) {
      servoObj.setDirection(nextState.getDirection().primitiveServoConversion());
    }
    if (noState || currentState.getPosition() != nextState.getPosition()) {
      servoObj.setPosition(nextState.getPosition());
    }
    nextState.makeCurrent();
  }
}
