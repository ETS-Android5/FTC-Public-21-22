package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;

import java.util.HashMap;
import java.util.Map;

public class ResetDcMotorPipe extends HardwarePipeline {
  private final Map<String, ResetState> resetStates = new HashMap<>();
  boolean allReset = false;

  public ResetDcMotorPipe(String name) {
    super(name);
  }

  public ResetDcMotorPipe(String name, HardwarePipeline nextPipe) {
    super(name, nextPipe);
  }

  @Override
  @SuppressWarnings("all")
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    if (!allReset) {
      hardware.forEach(
          (k, v) -> {
            if (!(resetStates.containsKey(k) && resetStates.get(k) == ResetState.RESET)
                && v instanceof DcMotor) {
              IMotorState motorState =
                  r.getNextMotorStates().stream()
                      .filter((m) -> m.getName().equals(k))
                      .findFirst()
                      .orElse(null);
              if (!resetStates.containsKey(k)) {
                ((DcMotor) v).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                resetStates.put(k, ResetState.RESETTING);
              } else {
                ((DcMotor) v).setMode(motorState.getRunMode().primitiveConversion());
                resetStates.put(k, ResetState.RESET);
              }
            }
          });
      if (InitializedFilterPipe.getInstance().everythingIsInitialized()) {
        if (resetStates.values().stream().allMatch((v) -> v == ResetState.RESET)) {
          allReset = true;
        }
      }
    }
    return super.process(hardware, r);
  }
}
