package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;

import java.util.List;
import java.util.stream.Collectors;

public class StateFilterResult {
  private final List<IMotorState> nextMotorStates;
  private final List<IServoState> nextServoStates;

  public StateFilterResult(Component component) {
    nextMotorStates =
        component.getNextState().stream()
            .filter((m) -> m instanceof IMotorState)
            .map((m) -> (IMotorState) m)
            .collect(Collectors.toList());
    nextServoStates =
        component.getNextState().stream()
            .filter((s) -> s instanceof IServoState)
            .map((s) -> (IServoState) s)
            .collect(Collectors.toList());
  }

  public StateFilterResult(List<IMotorState> motorStates, List<IServoState> servoStates) {
    this.nextMotorStates = motorStates;
    this.nextServoStates = servoStates;
  }

  public List<IMotorState> getNextMotorStates() {
    return nextMotorStates;
  }

  public List<IServoState> getNextServoStates() {
    return nextServoStates;
  }
}
