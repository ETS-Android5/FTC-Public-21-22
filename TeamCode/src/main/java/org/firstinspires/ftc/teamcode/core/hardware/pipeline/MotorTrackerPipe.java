package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;

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
  @SuppressWarnings("all")
  public StateFilterResult process(Map<String, Object> hardware, StateFilterResult r) {
    r.getNextMotorStates()
        .forEach(
            (m) -> {
              if (m.getRunMode() == RunMode.RUN_TO_POSITION
                  || m.getRunMode() == RunMode.RUN_USING_ENCODER) {
                int pos = ((DcMotor) hardware.get(m.getName())).getCurrentPosition();
                motorPositions.put(m.getName(), pos);
              }
            });
    Iterator<CallbackData> iter = motorPositionCallbacks.iterator();
    while (iter.hasNext()) {
      CallbackData data = iter.next();
      if (data == null) {
        iter.remove();
      } else {
        String motorName = data.getMotorName();
        if (motorPositions.containsKey(motorName)) {
          Integer motorPosition = motorPositions.get(motorName);
          int target = data.getMotorTargetPosition();
          int tolerance = data.getToleranceTicks();
          int minimum = target - tolerance;
          int maximum = target + tolerance;
          if (motorPosition != null && motorPosition >= minimum && motorPosition <= maximum) {
            data.getCallback().run();
            iter.remove();
          }
        }
      }
    }
    return super.process(hardware, r);
  }
}
