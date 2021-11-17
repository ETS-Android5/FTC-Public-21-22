package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;

import java.util.HashMap;
import java.util.Map;

public class ResetDcMotorPipe extends HardwarePipeline {
    public ResetDcMotorPipe(String name) {
        super(name);
    }

    public ResetDcMotorPipe(String name, HardwarePipeline nextPipe) {
        super(name, nextPipe);
    }

    private final Map<String, ResetState> resetStates = new HashMap<>();

    @Override
    public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    hardware.forEach(
        (k, v) -> {
          if (!(resetStates.containsKey(k) && resetStates.get(k) == ResetState.RESET) && v instanceof DcMotor) {
            IMotorState motorState =
                    r.getNextMotorStates().stream()
                        .filter((m) -> m.getName().equals(k))
                        .findFirst()
                        .orElse(null);
            if (motorState != null && resetStates.get(k) != ResetState.RESET) {
              if (!resetStates.containsKey(k)) {
                ((DcMotor) v).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                resetStates.put(k, ResetState.RESETTING);
              } else {
                ((DcMotor) v).setMode(motorState.getRunMode().primitiveConversion());
                resetStates.put(k, ResetState.RESET);
              }
            }
          }
        });
        return super.process(hardware, r);
    }

    private enum ResetState {
        RESETTING,
        RESET,
    }
}
