package org.firstinspires.ftc.teamcode.core.hardware.pipeline;

import java.util.function.Predicate;

public class CallbackData<T> {
  private final String hardwareName;
  private final Predicate<T> conditionsMet;
  private final Runnable callback;

  public CallbackData(String motorName, Predicate<T> conditionsMet, Runnable callback) {
    this.hardwareName = motorName;
    this.conditionsMet = conditionsMet;
    this.callback = callback;
  }

  public String getHardwareName() {
    return hardwareName;
  }

  public boolean conditionsMet(T in) {
    return conditionsMet.test(in);
  }

  public Runnable getCallback() {
    return callback;
  }
}
