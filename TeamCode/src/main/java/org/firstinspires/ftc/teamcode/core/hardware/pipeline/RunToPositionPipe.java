package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.state.IMotorState;
import org.firstinspires.ftc.teamcode.core.hardware.state.RunToPositionTracker;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class RunToPositionPipe extends HardwarePipeline {
  private final List<RunToPositionTracker> trackedMotors = new LinkedList<>();

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
                trackedMotors.add(
                    new RunToPositionTracker(
                        motorName, currentPosition, nextState.getTargetPosition()));
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
            if (motorObj instanceof DcMotorEx) {
              double power = motor.getTargetPowerPercentage(currentPosition);
              if (shouldAdjustForRealPower && power != 0) {
                double velocity = MotorTrackerPipe.getInstance().getVelocity(motorName);
                double maxVelocity = nextState.getPowerAndTickRateRelation().apply(1.0);
                double currentPercentPower = velocity / maxVelocity;
                if (currentPercentPower != power) {
                  boolean useStdPowerCorrectionIfPossible =
                      Math.abs(motor.getStartingTicks() - motor.getTargetTicks())
                          > nextState.getAdjustmentThreshold();
                  if ((useStdPowerCorrectionIfPossible && nextState.getPowerCorrection() != null)
                      || (!useStdPowerCorrectionIfPossible
                          && nextState.getAdjustmentPowerCorrectionCurve() != null)) {
                    double percentProgress;
                    if (motor.getStartingTicks() < motor.getTargetTicks()) {
                      percentProgress =
                          ((double) currentPosition - (double) motor.getStartingTicks())
                              / ((double) motor.getTargetTicks()
                                  - (double) motor.getStartingTicks());
                    } else {
                      percentProgress =
                          ((double) motor.getStartingTicks() - (double) currentPosition)
                              / ((double) motor.getStartingTicks()
                                  - (double) motor.getTargetTicks());
                    }
                    if (useStdPowerCorrectionIfPossible) {
                      power =
                          nextState
                              .getPowerCorrection()
                              .apply(currentPercentPower, power, percentProgress);
                    } else {
                      power =
                          nextState
                              .getAdjustmentPowerCorrectionCurve()
                              .apply(currentPercentPower, power, percentProgress);
                    }
                  } else {
                    power -= currentPercentPower - power;
                    power = power > 1 ? 1 : power < -1 ? -1 : power;
                  }
                }
              }
              ((DcMotorEx) motorObj).setPower(power);
              motor.setLastPower(power);
            }
          }
        });
    return super.process(hardware, r);
  }
}
