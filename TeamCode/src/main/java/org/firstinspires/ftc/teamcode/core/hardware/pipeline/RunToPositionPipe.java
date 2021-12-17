package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.RunToPositionTracker;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class RunToPositionPipe extends HardwarePipeline {
  private final List<RunToPositionTracker> trackedMotors = new LinkedList<>();

  public RunToPositionPipe(String name) {
    super(name);
  }

  public RunToPositionPipe(String name, HardwarePipeline nextElement) {
    super(name, nextElement);
  }

  @Override
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    r.getNextMotorStates()
        .forEach(
            (nextState) -> {
              String motorName = nextState.getName();
              IMotorState currentState = State.currentStateOf(motorName);
              if ((currentState == null || currentState.getRunMode() != RunMode.RUN_TO_POSITION)
                  && nextState.getRunMode() == RunMode.RUN_TO_POSITION) {
                int currentPosition = MotorTrackerPipe.getInstance().getPositionOf(motorName);
                TriFunction<Integer, Integer, Integer, Double> powerCurve =
                    nextState.getPowerCurve();
                if (powerCurve == null) {
                  powerCurve = PowerCurves.generatePowerCurve(1, 2.33);
                }
                trackedMotors.add(
                    new RunToPositionTracker(
                        motorName,
                        powerCurve,
                        currentPosition,
                        nextState.getTargetPosition(),
                        nextState.getPower()));
              } else if ((currentState == null
                      || currentState.getRunMode() == RunMode.RUN_TO_POSITION)
                  && nextState.getRunMode() != RunMode.RUN_TO_POSITION) {
                trackedMotors.removeIf((p) -> p.getName().equals(motorName));
              }
            });
    trackedMotors.forEach(
        (motor) -> {
          String motorName = motor.getName();
          IMotorState currentState = State.currentStateOf(motorName);
          IMotorState nextState = State.nextStateOf(motorName);
          int currentPosition = MotorTrackerPipe.getInstance().getPositionOf(motorName);
          if (currentState != null
              && nextState != null
              && currentState.getTargetPosition() != nextState.getTargetPosition()) {
            motor.mutateTo(currentPosition, nextState.getTargetPosition());
          }
          boolean shouldAdjustForRealPower = false;
          if (nextState != null) {
            shouldAdjustForRealPower = nextState.getPowerAndTickRateRelation() != null;
          }
          if (motor.shouldUpdatePower(currentPosition) || shouldAdjustForRealPower) {
            Object motorObj = hardware.get(motorName);
            if (motorObj instanceof DcMotor) {
              double power = motor.getTargetPowerPercentage(currentPosition);
              if (nextState != null) {
                double maxAcceleration = nextState.getMaxAcceleration();
                if (maxAcceleration != 0) {
                  double prevPower = motor.getLastPower();
                  if (Math.abs(power - prevPower) > maxAcceleration) {
                    power =
                        power < prevPower
                            ? prevPower - maxAcceleration
                            : prevPower + maxAcceleration;
                  }
                }
              }
              if (shouldAdjustForRealPower && power != 0) {
                double velocity = MotorTrackerPipe.getInstance().getVelocity(motorName);
                double maxVelocity = nextState.getPowerAndTickRateRelation().apply(1.0);
                double currentPercentPower = velocity / maxVelocity;
                Log.d("WTF", "CURRENT PERCENT POWER IS " + currentPercentPower);
                Log.d("WTF", "TARGET POWER IS " + power);
                Log.d("WTF", "ADJUSTING POWER TO " + (power - (currentPercentPower - power)));
                power -= currentPercentPower - power;
              }
              power = power > 1 ? 1 : power < -1 ? -1 : power;
              ((DcMotor) motorObj).setPower(power);
              motor.setLastPower(power);
            }
          }
        });
    return super.process(hardware, r);
  }
}
