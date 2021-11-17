package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
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
    private final List<CallbackData> motorPositionCallbacks = new LinkedList<>();

    public void setCallbackForMotorPosition(CallbackData data) {
        motorPositionCallbacks.add(data);
    }

    public int getPositionOf(String motorName) throws IllegalArgumentException {
        Integer motorPosition = motorPositions.get(motorName);
        if (motorPosition != null) {
            return motorPosition;
        }
        throw new IllegalArgumentException("That Motor is not tracked!");
    }

    @Override
    public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
        hardware.forEach((k, v) -> {
            if (v instanceof DcMotor) {
                IMotorState motorState = r.getNextMotorStates().stream()
                        .filter((s) -> s.getName().equals(k)).findFirst().orElse(null);
                if (motorState != null) {
                    RunMode runMode = motorState.getRunMode();
                    if (runMode == RunMode.RUN_TO_POSITION || runMode == RunMode.RUN_USING_ENCODER) {
                        motorPositions.put(k, ((DcMotor) v).getCurrentPosition());
                    }
                }
            }
        });
        Iterator<CallbackData> iter = motorPositionCallbacks.iterator();
        while (iter.hasNext()) {
            CallbackData data = iter.next();
            if (data != null && motorPositions.containsKey(data.getMotorName())) {
                Integer motorPosition = motorPositions.get(data.getMotorName());
                int minimum = data.getMotorTargetPosition() - data.getToleranceTicks();
                int maximum = data.getMotorTargetPosition() + data.getToleranceTicks();
                if (motorPosition != null && motorPosition >= minimum && motorPosition <= maximum) {
                    data.getCallback().run();
                    iter.remove();
                }
            }
        }
        return super.process(hardware, r);
    }
}
