package org.firstinspires.ftc.teamcode.core.hardware.state;

public class CachedValue<T> {
  private final T value;
  private final long id;

  public CachedValue(T value, long id) {
    this.value = value;
    this.id = id;
  }

  public T getValue() {
    return value;
  }

  public long getId() {
    return id;
  }
}
