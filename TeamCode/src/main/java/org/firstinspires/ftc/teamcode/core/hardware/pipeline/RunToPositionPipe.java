package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
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

    private final List<RunToPositionTracker> trackedMotors = new LinkedList<>();

    @Override
    public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
        r.getNextMotorStates().forEach((nextState) -> {
            String motorName = nextState.getName();
            IMotorState currentState = State.currentStateOf(motorName);
            if ((currentState == null || currentState.getRunMode() != RunMode.RUN_TO_POSITION)
                    && nextState.getRunMode() == RunMode.RUN_TO_POSITION) {
                try {
                    int currentPosition = MotorTrackerPipe.getInstance().getPositionOf(motorName);
                    TriFunction<Integer, Integer, Integer, Double> powerCurve = nextState.getPowerCurve();
                    if (powerCurve == null) {
                        powerCurve = PowerCurves.RUN_TO_POSITION_RAMP;
                    }
                    trackedMotors.add(new RunToPositionTracker(
                            motorName,
                            powerCurve,
                            currentPosition,
                            nextState.getTargetPosition(),
                            nextState.getPower()));
                } catch (IllegalArgumentException ignored) {}
            } else if ((currentState == null || currentState.getRunMode() == RunMode.RUN_TO_POSITION)
                    && nextState.getRunMode() != RunMode.RUN_TO_POSITION) {
                trackedMotors.removeIf((p) -> p.getName().equals(motorName));
            }
        });
    trackedMotors.forEach(
        (motor) -> {
          String motorName = motor.getName();
          IMotorState currentState = State.currentStateOf(motorName);
          IMotorState nextState = State.nextStateOf(motorName);
          if (currentState != null && nextState != null) {
            if (currentState.getTargetPosition() != nextState.getTargetPosition()) {
              motor.mutateTo(
                  MotorTrackerPipe.getInstance().getPositionOf(motorName),
                  nextState.getTargetPosition());
            }
            double power =
                motor.getTargetPowerPercentage(
                    MotorTrackerPipe.getInstance().getPositionOf(motorName));
            if (motor.shouldUpdatePower()) {
              Object motorObj = hardware.get(motorName);
              if (motorObj instanceof DcMotor) {
                  Log.d("MTP", "SETTING POWER OF " + motorName + " TO: " + power);
                ((DcMotor) motorObj).setPower(power);
              }
            }
          }
        });
        return super.process(hardware, r);
    }
}
