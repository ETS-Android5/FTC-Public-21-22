package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;

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
        r.getNextMotorStates().forEach((m) -> {
            if (m.getRunMode() == RunMode.RUN_TO_POSITION || m.getRunMode() == RunMode.RUN_USING_ENCODER) {
                Object motor = hardware.get(m.getName());
                if (motor != null) {
                    int pos = ((DcMotor) motor).getCurrentPosition();
                    Log.d("MTP", "FETCHING CURRENT POSITION OF " + m.getName() + ": " + pos);
                    motorPositions.put(m.getName(), pos);
                }
            }
        });
        Iterator<CallbackData> iter = motorPositionCallbacks.iterator();
        while (iter.hasNext()) {
            Log.d("MTP", "MTP ITER");
            CallbackData data = iter.next();
            if (data != null && motorPositions.containsKey(data.getMotorName())) {
                Log.d("MTP", "Callback for " + data.getMotorName());
                Integer motorPosition = motorPositions.get(data.getMotorName());
                int minimum = data.getMotorTargetPosition() - data.getToleranceTicks();
                int maximum = data.getMotorTargetPosition() + data.getToleranceTicks();
                Log.d("MTP", "CURRENT POSITION: " + motorPosition);
                Log.d("MTP", "TARGET: " + data.getMotorTargetPosition());
                Log.d("MTP", "MIN: " + minimum);
                Log.d("MTP", "MAX: " + maximum);
                if (motorPosition != null && motorPosition >= minimum && motorPosition <= maximum) {
                    data.getCallback().run();
                    iter.remove();
                }
            }
        }
        return super.process(hardware, r);
    }
}
