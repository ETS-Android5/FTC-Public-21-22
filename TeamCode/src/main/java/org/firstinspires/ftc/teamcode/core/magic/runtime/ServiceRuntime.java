package org.firstinspires.ftc.teamcode.core.magic.runtime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Namable;
import org.firstinspires.ftc.teamcode.core.annotations.Observable;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class ServiceRuntime implements ReflectionBasedMagicRuntime {
  private final Telemetry telemetry;
  private final ConcurrentHashMap<String, Func<?>> observables;
  private final Object[] observableObjects;

  public ServiceRuntime(Telemetry telemetry, Object... observableObjects) {
    this.telemetry = telemetry;
    observables = new ConcurrentHashMap<>();
    this.observableObjects = observableObjects;
  }

  @Override
  public void initialize() {
    for (Object obj : observableObjects) {
      findInjectableFields(obj);
    }
  }

  @Override
  public void waveWand() {
    new Thread(
            () -> observables.forEachEntry(5, (e) -> telemetry.addData(e.getKey(), e.getValue())))
        .start();
  }

  private void findInjectableFields(Object root) {
    if (root == null || Arrays.asList(Constants.BOXING_CONVERSIONS).contains(root.getClass())) {
      return;
    }
    List<Field> fields = Arrays.asList(root.getClass().getFields());
    List<Method> methods = Arrays.asList(root.getClass().getMethods());

    fields.forEach(f -> f.setAccessible(true));
    methods.forEach(m -> m.setAccessible(true));

    List<Field> deepFields =
        fields.stream()
            .filter((f) -> !Arrays.asList(Constants.BOXING_CONVERSIONS).contains(f.getType()))
            .collect(Collectors.toList());
    deepFields.forEach(
        f -> {
          try {
            findInjectableFields(f.get(root));
          } catch (IllegalAccessException ignored) {
          }
        });

    List<Field> observableFields =
        fields.stream()
            .filter((f) -> f.isAnnotationPresent(Observable.class))
            .collect(Collectors.toList());
    List<Method> observableMethods =
        methods.stream()
            .filter(m -> m.isAnnotationPresent(Observable.class))
            .collect(Collectors.toList());

    String srcName = root instanceof Namable ? ((Namable) root).getName() + "/" : "";
    observableFields.forEach(
        (f) -> {
          Observable fieldAnnotation = f.getAnnotation(Observable.class);
          observables.put(
              srcName + (fieldAnnotation != null ? fieldAnnotation.key() : ""),
              () -> {
                try {
                  f.setAccessible(true);
                  Object result = f.get(root);
                  return result == null ? "null" : result.toString();
                } catch (IllegalAccessException ignored) {
                  return "IllegalAccess";
                }
              });
        });
    observableMethods.forEach(
        m -> {
          Observable methodAnnotation = m.getAnnotation(Observable.class);
          observables.put(
              srcName + (methodAnnotation != null ? methodAnnotation.key() : ""),
              () -> {
                try {
                  m.setAccessible(true);
                  Object result = m.invoke(root);
                  return result == null ? "null" : result.toString();
                } catch (IllegalAccessException | InvocationTargetException ignored) {
                  return "IllegalAccess";
                }
              });
        });
  }
}
