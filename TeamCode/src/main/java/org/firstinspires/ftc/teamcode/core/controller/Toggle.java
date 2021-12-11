package org.firstinspires.ftc.teamcode.core.controller;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class Toggle {
  private final Supplier<Boolean> valueRetriever;
  private final List<Runnable> onPressed = new LinkedList<>();
  private final List<Runnable> onPressedDebounced = new LinkedList<>();
  private final List<Runnable> onReleased = new LinkedList<>();
  private final List<Runnable> onReleasedDebounced = new LinkedList<>();
  private boolean previousValue;
  private boolean currentValue;

  public Toggle(Supplier<Boolean> valueRetriever) {
    this.valueRetriever = valueRetriever;
    this.previousValue = valueRetriever.get();
    this.currentValue = valueRetriever.get();
  }

  boolean currentValue() {
    return currentValue;
  }

  void update() {
    previousValue = currentValue;
    currentValue = valueRetriever.get();
    executeCallbacks();
  }

  void registerOnPressedCallback(Runnable callback, boolean debounced) {
    if (debounced) {
      onPressedDebounced.add(callback);
    } else {
      onPressed.add(callback);
    }
  }

  void registerOnReleasedCallback(Runnable callback, boolean debounced) {
    if (debounced) {
      onReleasedDebounced.add(callback);
    } else {
      onReleased.add(callback);
    }
  }

  private void executeCallbacks() {
    if (currentValue) {
      onPressed.forEach(Runnable::run);
    } else {
      onReleased.forEach(Runnable::run);
    }
    if (currentValue && !previousValue) {
      onPressedDebounced.forEach(Runnable::run);
    }
    if (!currentValue && previousValue) {
      onReleasedDebounced.forEach(Runnable::run);
    }
  }
}
