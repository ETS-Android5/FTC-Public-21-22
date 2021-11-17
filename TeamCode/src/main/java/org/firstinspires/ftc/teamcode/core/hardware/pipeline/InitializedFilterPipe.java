package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class InitializedFilterPipe extends HardwarePipeline {
  public InitializedFilterPipe(String name) {
    super(name);
  }

  public InitializedFilterPipe(String name, HardwarePipeline nextPipe) {
    super(name, nextPipe);
  }

  private boolean allInitialized = false;

  @Override
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    if (!allInitialized) {
        List<IMotorState> initializedMotors = r.getNextMotorStates().stream()
                .filter((m) -> hardware.containsKey(m.getName()))
                .collect(Collectors.toList());
        List<IServoState> initializedServos = r.getNextServoStates().stream()
                .filter((s) -> hardware.containsKey(s.getName()))
                .collect(Collectors.toList());
        if (initializedMotors.size() == r.getNextMotorStates().size() && initializedServos.size() == r.getNextServoStates().size()) {
            allInitialized = true;
        }
        return super.process(hardware, new StateFilterResult(initializedMotors, initializedServos));
    } else {
        return super.process(hardware, r);
    }
  }
}
