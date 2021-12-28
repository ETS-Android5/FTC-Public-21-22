package org.firstinspires.ftc.teamcode.core.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class Controller implements Namable {
  private final String name;
  private final List<Toggle> toggleSurfaces = new LinkedList<>();
  private Gamepad gamepad;
  private Toggle leftBumper;
  private Toggle rightBumper;
  private Toggle leftStick;
  private Toggle rightStick;
  private Toggle dpadDown;
  private Toggle dpadRight;
  private Toggle dpadLeft;
  private Toggle dpadUp;
  private Toggle a;
  private Toggle b;
  private Toggle x;
  private Toggle y;
  private Toggle back;
  private Toggle start;
  private Function<Double, Double> leftTriggerManipulation = null;
  private Function<Double, Double> rightTriggerManipulation = null;
  private Function<Double, Double> leftStickXManipulation = null;
  private Function<Double, Double> rightStickXManipulation = null;
  private Function<Double, Double> leftStickYManipulation = null;
  private Function<Double, Double> rightStickYManipulation = null;

  public Controller(String name) {
    this.name = name;
  }

  private static double DEFAULT_MANIPULATION(double in) {
    return in;
  }

  public void initialize(Gamepad gamepad) {
    this.gamepad = gamepad;
    leftBumper = new Toggle(() -> gamepad.left_bumper);
    rightBumper = new Toggle(() -> gamepad.right_bumper);
    leftStick = new Toggle(() -> gamepad.left_stick_button);
    rightStick = new Toggle(() -> gamepad.right_stick_button);
    dpadDown = new Toggle(() -> gamepad.dpad_down);
    dpadRight = new Toggle(() -> gamepad.dpad_right);
    dpadLeft = new Toggle(() -> gamepad.dpad_left);
    dpadUp = new Toggle(() -> gamepad.dpad_up);
    back = new Toggle(() -> gamepad.back);
    start = new Toggle(() -> gamepad.start);

    a = new Toggle(() -> gamepad.a || gamepad.cross);
    b = new Toggle(() -> gamepad.b || gamepad.circle);
    x = new Toggle(() -> gamepad.x || gamepad.square);
    y = new Toggle(() -> gamepad.y || gamepad.triangle);

    toggleSurfaces.addAll(
        Arrays.asList(
            leftBumper,
            rightBumper,
            leftStick,
            rightStick,
            dpadDown,
            dpadRight,
            dpadLeft,
            dpadUp,
            a,
            b,
            x,
            y,
            back,
            start));
  }

  @Override
  public String getName() {
    return name;
  }

  public void setManipulation(Function<Double, Double> manipulation, ScalarSurface scalarSurface) {
    switch (scalarSurface) {
      case LEFT_TRIGGER:
        leftTriggerManipulation = manipulation;
        break;
      case RIGHT_TRIGGER:
        rightTriggerManipulation = manipulation;
        break;
      case LEFT_STICK_X:
        leftStickXManipulation = manipulation;
        break;
      case RIGHT_STICK_X:
        rightStickXManipulation = manipulation;
        break;
      case LEFT_STICK_Y:
        leftStickYManipulation = manipulation;
        break;
      case RIGHT_STICK_Y:
        rightStickYManipulation = manipulation;
        break;
    }
  }

  public double leftTrigger() {
    return Optional.ofNullable(leftTriggerManipulation)
        .orElse(Controller::DEFAULT_MANIPULATION)
        .apply((double) gamepad.left_trigger);
  }

  public double rightTrigger() {
    return Optional.ofNullable(rightTriggerManipulation)
        .orElse(Controller::DEFAULT_MANIPULATION)
        .apply((double) gamepad.right_trigger);
  }

  public double leftStickX() {
    return Optional.ofNullable(leftStickXManipulation)
        .orElse(Controller::DEFAULT_MANIPULATION)
        .apply((double) gamepad.left_stick_x);
  }

  public double rightStickX() {
    return Optional.ofNullable(rightStickXManipulation)
        .orElse(Controller::DEFAULT_MANIPULATION)
        .apply((double) gamepad.right_stick_x);
  }

  public double leftStickY() {
    return Optional.ofNullable(leftStickYManipulation)
        .orElse(Controller::DEFAULT_MANIPULATION)
        .apply((double) gamepad.left_stick_y * -1);
  }

  public double rightStickY() {
    return Optional.ofNullable(rightStickYManipulation)
        .orElse(Controller::DEFAULT_MANIPULATION)
        .apply((double) gamepad.right_stick_y * -1);
  }

  public boolean leftBumper() {
    return leftBumper.currentValue();
  }

  public boolean rightBumper() {
    return rightBumper.currentValue();
  }

  public boolean leftStick() {
    return leftStick.currentValue();
  }

  public boolean rightStick() {
    return rightStick.currentValue();
  }

  public boolean dpadDown() {
    return dpadDown.currentValue();
  }

  public boolean dpadRight() {
    return dpadRight.currentValue();
  }

  public boolean dpadLeft() {
    return dpadLeft.currentValue();
  }

  public boolean dpadUp() {
    return dpadUp.currentValue();
  }

  public boolean a() {
    return a.currentValue();
  }

  public boolean b() {
    return b.currentValue();
  }

  public boolean x() {
    return x.currentValue();
  }

  public boolean y() {
    return y.currentValue();
  }

  public boolean back() {
    return back.currentValue();
  }

  public boolean start() {
    return start.currentValue();
  }

  public void update() {
    toggleSurfaces.forEach(Toggle::update);
  }

  public void registerOnPressedCallback(
      Runnable callback, boolean debounced, BooleanSurface surface) {
    Optional.ofNullable(toggleForControlSurface(surface))
        .ifPresent((t) -> t.registerOnPressedCallback(callback, debounced));
  }

  public void registerOnReleasedCallback(
      Runnable callback, boolean debounced, BooleanSurface surface) {
    Optional.ofNullable(toggleForControlSurface(surface))
        .ifPresent((t) -> t.registerOnReleasedCallback(callback, debounced));
  }

  public void vibrate(double leftPower, double rightPower, int durationMs) {
    gamepad.rumble(leftPower, rightPower, durationMs);
  }

  public void vibrate(int durationMs) {
    gamepad.rumble(durationMs);
  }

  private Toggle toggleForControlSurface(BooleanSurface surface) {
    switch (surface) {
      case LEFT_BUMPER:
        return leftBumper;
      case RIGHT_BUMPER:
        return rightBumper;
      case LEFT_STICK:
        return leftStick;
      case RIGHT_STICK:
        return rightStick;
      case DPAD_DOWN:
        return dpadDown;
      case DPAD_RIGHT:
        return dpadRight;
      case DPAD_LEFT:
        return dpadLeft;
      case DPAD_UP:
        return dpadUp;
      case A:
        return a;
      case B:
        return b;
      case X:
        return x;
      case Y:
        return y;
      case BACK:
        return back;
      case START:
        return start;
    }
    return null;
  }
}
