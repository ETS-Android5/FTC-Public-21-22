package org.firstinspires.ftc.teamcode.core.magic.runtime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.Namable;
import org.firstinspires.ftc.teamcode.core.annotations.PostInit;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.LateInit;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class AotRuntime implements HardwareMapDependentReflectionBasedMagicRuntime {
  private static final Class<?>[] KNOWN_INJECTABLE_HARDWARE =
      new Class<?>[] {
        DcMotorEx.class,
        Servo.class,
        CRServo.class,
        WebcamName.class,
        Rev2mDistanceSensor.class,
        BNO055IMU.class,
        AnalogInput.class,
        DigitalChannel.class
      };

  private static final Class<?>[][] HARDWARE_PRIORITY_GROUPS =
      new Class<?>[][] {
        {
          DcMotorEx.class,
        },
        {
          Servo.class, CRServo.class,
        },
        {
          WebcamName.class,
          Rev2mDistanceSensor.class,
          BNO055IMU.class,
          AnalogInput.class,
          DigitalChannel.class
        },
      };

  private final Object robotObject;
  private final ConcurrentHashMap<String, Object> initializedObjects;
  private final ConcurrentHashMap<String, Runnable> lateInitializables;
  private final List<AnnotationPair> hardwareFields;
  private final boolean isAutonomous;
  private HardwareMap hardwareMap;

  public AotRuntime(
      Object robotObject,
      ConcurrentHashMap<String, Object> initializedObjects,
      boolean isAutonomous) {
    this.robotObject = robotObject;
    this.initializedObjects = initializedObjects;
    this.lateInitializables = new ConcurrentHashMap<>();
    this.hardwareFields = new LinkedList<>();
    this.isAutonomous = isAutonomous;
  }

  @Override
  public void setHardwareMap(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
  }

  @Override
  public void lateInit(String deviceName) {
    Runnable initFunc = lateInitializables.get(deviceName);
    if (initFunc != null) {
      initFunc.run();
      lateInitializables.remove(deviceName);
    }
  }

  @Override
  public void initialize() {
    hardwareFields.addAll(getInjectableFields(robotObject, true));
  }

  @Override
  public void waveWand() {
    new Thread(() -> hardwareFields.forEach(this::injectField)).start();
  }

  private List<AnnotationPair> getInjectableFields(Object targetObject, boolean sort) {
    if (targetObject == null
        || Arrays.asList(Constants.BOXING_CONVERSIONS).contains(targetObject.getClass())) {
      return new LinkedList<>();
    }
    List<AnnotationPair> injectableFields = new LinkedList<>();
    Field[] objFields = targetObject.getClass().getFields();
    for (Field objField : objFields) {
      objField.setAccessible(true);
      if (Arrays.asList(Constants.BOXING_CONVERSIONS).contains(objField.getType())) {
        continue;
      }
      if (objField.isAnnotationPresent(AutonomousOnly.class) && !isAutonomous) continue;
      if (objField.isAnnotationPresent(Hardware.class)
          && objField.isAnnotationPresent(LateInit.class)) {
        lateInitializables.put(
            Objects.requireNonNull(objField.getAnnotation(Hardware.class)).name(),
            () ->
                injectField(
                    new AnnotationPair(
                        objField.getAnnotation(Hardware.class), targetObject, objField)));
      } else if (objField.isAnnotationPresent(Hardware.class)) {
        injectableFields.add(
            new AnnotationPair(objField.getAnnotation(Hardware.class), targetObject, objField));
      } else {
        try {
          Optional.ofNullable(objField.get(targetObject))
              .ifPresent((o) -> injectableFields.addAll(getInjectableFields(o, false)));
        } catch (IllegalAccessException ignored) {
        }
      }
    }
    List<AnnotationPair> processedFields =
        injectableFields.stream()
            .filter(
                i ->
                    Arrays.asList(KNOWN_INJECTABLE_HARDWARE)
                        .contains(i.getAnnotationTargetField().getType()))
            .collect(Collectors.toList());
    if (sort) {
      return sortByPriorityGroups(processedFields);
    }
    return processedFields;
  }

  private void injectField(AnnotationPair field) {
    Hardware annotation = (Hardware) field.getAnnotation();
    Class<?> hardwareObj = field.getAnnotationTargetField().getType();
    if (hardwareObj == DcMotorEx.class) {
      DcMotorEx motor = hardwareMap.get(DcMotorEx.class, annotation.name());
      motor.setDirection(annotation.direction().primitiveConversion());
      motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      motor.setMode(annotation.runMode().primitiveConversion());
      motor.setZeroPowerBehavior(annotation.zpBehavior().primitiveConversion());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), motor);
        initializedObjects.put(annotation.name(), motor);
        completePostInit(field, motor, DcMotorEx.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == Servo.class) {
      Servo servo = hardwareMap.servo.get(annotation.name());
      servo.setDirection(annotation.direction().primitiveServoConversion());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), servo);
        initializedObjects.put(annotation.name(), servo);
        completePostInit(field, servo, Servo.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == CRServo.class) {
      CRServo servo = hardwareMap.crservo.get(annotation.name());
      servo.setDirection(annotation.direction().primitiveConversion());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), servo);
        initializedObjects.put(annotation.name(), servo);
        completePostInit(field, servo, CRServo.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == WebcamName.class) {
      WebcamName webcamName = hardwareMap.get(WebcamName.class, annotation.name());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), webcamName);
        completePostInit(field, webcamName, WebcamName.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == Rev2mDistanceSensor.class
        && field.getAnnotationContainer() instanceof Namable) {
      String name = ((Namable) field.getAnnotationContainer()).getName();
      Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, name);
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), sensor);
        initializedObjects.put(name, field.getAnnotationContainer());
        completePostInit(field, sensor, Rev2mDistanceSensor.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == BNO055IMU.class) {
      BNO055IMU sensor = hardwareMap.get(BNO055IMU.class, annotation.name());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), sensor);
        initializedObjects.put(annotation.name(), field.getAnnotationContainer());
        completePostInit(field, sensor, BNO055IMU.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == AnalogInput.class
        && field.getAnnotationContainer() instanceof Namable) {
      String name = ((Namable) field.getAnnotationContainer()).getName();
      AnalogInput analogInput = hardwareMap.analogInput.get(name);
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), analogInput);
        initializedObjects.put(name, field.getAnnotationContainer());
        completePostInit(field, analogInput, AnalogInput.class);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == DigitalChannel.class) {
      DigitalChannel channel = hardwareMap.digitalChannel.get(annotation.name());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), channel);
        initializedObjects.put(annotation.name(), channel);
        completePostInit(field, channel, DigitalChannel.class);
      } catch (IllegalAccessException ignored) {
      }
    }
  }

  private void completePostInit(AnnotationPair field, Object initialized, Class<?> argType) {
    for (Method m : field.getAnnotationContainer().getClass().getMethods()) {
      if (m.isAnnotationPresent(PostInit.class) && m.getReturnType().equals(Void.TYPE)) {
        // We should really be checking the parameter number and type but Android API <26 doesn't
        // support that
        // And our min API level is 24 and could only move up to 25
        // So we're going to use the mess around and find out method and just assume it's safe
        if (Objects.requireNonNull(m.getAnnotation(PostInit.class)).argType().equals(argType)) {
          try {
            m.invoke(field.getAnnotationContainer(), initialized);
          } catch (IllegalAccessException | InvocationTargetException ignored) {
          }
        }
      }
    }
  }

  // Could be modified quick or merge sort but I'm lazy
  // So it's O((i + 1) * logBase(5, n)^iteration )
  // Where i is number of categories
  private List<AnnotationPair> sortByPriorityGroups(List<AnnotationPair> pairs) {
    List<List<AnnotationPair>> groups = new LinkedList<>();
    for (Class<?>[] hardwarePriorityGroup : HARDWARE_PRIORITY_GROUPS) {
      List<AnnotationPair> hardware =
          pairs.stream()
              .filter(
                  a ->
                      Arrays.asList(hardwarePriorityGroup)
                          .contains(a.getAnnotationTargetField().getType()))
              .collect(Collectors.toList());
      groups.add(hardware);
      pairs.removeIf(hardware::contains);
    }
    // Put any items outside priority groups in
    if (pairs.size() > 0) {
      groups.add(new LinkedList<>(pairs));
    }
    List<AnnotationPair> sortedFlatList = new LinkedList<>();
    for (List<AnnotationPair> group : groups) {
      sortedFlatList.addAll(group);
    }
    return sortedFlatList;
  }
}
