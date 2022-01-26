package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.RunToPositionTracker;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class RunToPositionPipe extends HardwarePipeline {
    public RunToPositionPipe(String name) {
        super(name);
    }

    public RunToPositionPipe(String name, HardwarePipeline nextElement) {
        super(name, nextElement);
    }

    private final List<Pair<DcMotor, RunToPositionTracker>> trackedMotors = new LinkedList<>();

    @Override
    public Component process(Map<String, Object> hardware, Component c) {
        List<IMotorState> motors = c.getNextState().stream()
                .filter((s) -> s instanceof IMotorState)
                .map((s) -> (IMotorState) s)
                .collect(Collectors.toList());
        motors.forEach((nextState) -> {
            IMotorState currentState = State.currentStateOf(nextState.getName());
            DcMotor motorObj = (DcMotor) hardware.get(nextState.getName());
            if (motorObj != null) {
                if ((currentState == null || currentState.getRunMode() != RunMode.RUN_TO_POSITION)
                        && nextState.getRunMode() == RunMode.RUN_TO_POSITION) {
                    trackedMotors.add(new Pair<>(
                            motorObj,
                            new RunToPositionTracker(nextState.getName(), motorObj.getCurrentPosition(),
                                    nextState.getTargetPosition())
                    ));
                } else if ((currentState == null || currentState.getRunMode() == RunMode.RUN_TO_POSITION)
                        && nextState.getRunMode() != RunMode.RUN_TO_POSITION) {
                    trackedMotors.removeIf((p) -> p.first.equals(motorObj));
                }
            }
        });
        trackedMotors.forEach((p) -> {
            IMotorState currentState = State.currentStateOf(p.second.getName());
            IMotorState nextState = State.nextStateOf(p.second.getName());
            if (currentState != null && nextState != null) {
                if (currentState.getTargetPosition() != nextState.getTargetPosition()) {
                    p.second.mutateTo(p.first.getCurrentPosition(), nextState.getTargetPosition());
                }
                double power = p.second.getTargetPowerPercentage(p.first.getCurrentPosition());
                Log.d("WTF", "SET POWER: " + power);
                p.first.setPower(power);
            }
        });
        return super.process(hardware, c);
    }
}
