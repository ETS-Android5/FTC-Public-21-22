package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.HashMap;
import java.util.Map;

public class MotorTrackerPipe extends HardwarePipeline {
    private static MotorTrackerPipe instance;

    public static MotorTrackerPipe getInstance() {
        return instance;
    }

    public MotorTrackerPipe(String name) {
        super(name);
        MotorTrackerPipe.instance = this;
    }

    public MotorTrackerPipe(String name, HardwarePipeline nextPipe) {
        super(name, nextPipe);
        MotorTrackerPipe.instance = this;
    }

    private final Map<String, Integer> motorPositions = new HashMap<>();

    public int getPositionOf(String motorName) throws IllegalArgumentException {
        Integer motorPosition = motorPositions.get(motorName);
        if (motorPosition != null) {
            return motorPosition;
        }
        throw new IllegalArgumentException("That Motor is not tracked!");
    }

    @Override
    public Component process(Map<String, Object> hardware, Component c) {
        hardware.forEach((k, v) -> {
            if (v instanceof DcMotor) {
                IMotorState motorState = (IMotorState) c.getNextState().stream()
                        .filter((s) -> ((State) s).getName().equals(k)).findFirst().orElse(null);
                if (motorState != null) {
                    RunMode runMode = motorState.getRunMode();
                    if (runMode == RunMode.RUN_TO_POSITION || runMode == RunMode.RUN_USING_ENCODER) {
                        motorPositions.put(k, ((DcMotor) v).getCurrentPosition());
                    }
                }
            }
        });
        return super.process(hardware, c);
    }
}
