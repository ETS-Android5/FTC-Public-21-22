package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.Namable;

import java.util.HashMap;
import java.util.Map;

public class State implements Namable {
  private static final Map<String, State> currentStates = new HashMap<>();
  private static final Map<String, State> nextStates = new HashMap<>();
  String name;

  State(String name) {
    this.name = name;
    if (nextStates.containsKey(name)) {
      nextStates.replace(name, this);
    } else {
      nextStates.put(name, this);
    }
  }

  public static void clear() {
    currentStates.clear();
    nextStates.clear();
  }

  public static <T> T currentStateOf(String key) {
    return safeCast(currentStates.get(key));
  }

  public static <T> T nextStateOf(String key) {
    return safeCast(nextStates.get(key));
  }

  @SuppressWarnings("unchecked")
  private static <T> T safeCast(State s) {
    try {
      return s != null ? (T) s : null;
    } catch (ClassCastException e) {
      return null;
    }
  }

  public void makeCurrent() {
    nextStates.remove(name);
    if (currentStates.containsKey(name)) {
      currentStates.replace(name, this);
    } else {
      currentStates.put(name, this);
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
