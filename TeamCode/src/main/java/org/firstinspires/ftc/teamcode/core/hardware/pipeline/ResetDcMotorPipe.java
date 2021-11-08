package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

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
    public Component process(Map<String, Object> hardware, Component c) {
        hardware.forEach((k, v) -> {
            if (v instanceof DcMotor) {
                IMotorState motorState = (IMotorState) c.getNextState().stream()
                        .filter((s) -> ((State) s).getName().equals(k)).findFirst().orElse(null);
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
        return super.process(hardware, c);
    }

    private enum ResetState {
        RESETTING,
        RESET,
    }
}
