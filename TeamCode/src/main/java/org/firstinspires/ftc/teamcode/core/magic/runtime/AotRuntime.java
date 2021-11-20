package org.firstinspires.ftc.teamcode.core.magic.runtime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class AotRuntime implements HardwareMapDependentReflectionBasedMagicRuntime {
  private static final Class<?>[] KNOWN_INJECTABLE_HARDWARE =
      new Class<?>[] {DcMotor.class, Servo.class, WebcamName.class};

  private static final Class<?>[][] HARDWARE_PRIORITY_GROUPS =
      new Class<?>[][] {
        {
          DcMotor.class,
        },
        {
          Servo.class,
        },
        {WebcamName.class}
      };

  private final Object robotObject;
  private HardwareMap hardwareMap;
  private final ConcurrentHashMap<String, Object> initializedObjects;
  private final List<AnnotationPair> hardwareFields;
  private final boolean isAutonomous;

  public AotRuntime(
      Object robotObject,
      ConcurrentHashMap<String, Object> initializedObjects,
      boolean isAutonomous) {
    this.robotObject = robotObject;
    this.initializedObjects = initializedObjects;
    this.hardwareFields = new LinkedList<>();
    this.isAutonomous = isAutonomous;
  }

  @Override
  public HardwareMap getHardwareMap() {
    return hardwareMap;
  }

  @Override
  public void setHardwareMap(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
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
      if (objField.isAnnotationPresent(Hardware.class)) {
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
    if (hardwareObj == DcMotor.class) {
      DcMotor motor = hardwareMap.dcMotor.get(annotation.name());
      motor.setDirection(annotation.direction().primitiveConversion());
      motor.setMode(annotation.runMode().primitiveConversion());
      motor.setZeroPowerBehavior(annotation.zpBehavior().primitiveConversion());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), motor);
        initializedObjects.put(annotation.name(), motor);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == Servo.class) {
      Servo servo = hardwareMap.servo.get(annotation.name());
      servo.setDirection(annotation.direction().primitiveServoConversion());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), servo);
        initializedObjects.put(annotation.name(), servo);
      } catch (IllegalAccessException ignored) {
      }
    } else if (hardwareObj == WebcamName.class) {
      WebcamName webcamName = hardwareMap.get(WebcamName.class, annotation.name());
      try {
        field.getAnnotationTargetField().set(field.getAnnotationContainer(), webcamName);
      } catch (IllegalAccessException ignored) {
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
