package org.firstinspires.ftc.teamcode.core.hardware.state;

import android.util.Log;

import org.firstinspires.ftc.teamcode.core.hardware.pipeline.CallbackData;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;

public class MotorPositionReachedCallback {
  private final String motorName;
  private final int motorTarget;
  private final int motorTolerance;
  private final boolean targetAlreadyReached;

  public MotorPositionReachedCallback(
      String motorName, int motorTarget, int motorTolerance, boolean targetAlreadyReached) {
    this.motorName = motorName;
    this.motorTarget = motorTarget;
    this.motorTolerance = motorTolerance;
    this.targetAlreadyReached = targetAlreadyReached;
  }

  public void andThen(Runnable r) {
    if (targetAlreadyReached) {
      r.run();
    } else {
      MotorTrackerPipe.getInstance()
          .setCallbackForMotorPosition(
              new CallbackData<>(
                  motorName,
                  (ticks) -> {
                    Log.d("HERE", "" + ticks);
                    boolean ret = Math.abs(ticks - motorTarget) <= motorTolerance;
                    Log.d("HERE", "RET: " + ret);
                    return ret;
                  },
                  r));
    }
  }
}
