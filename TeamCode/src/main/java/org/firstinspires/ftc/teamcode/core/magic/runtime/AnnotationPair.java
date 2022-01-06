package org.firstinspires.ftc.teamcode.core.magic.runtime;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

public class AnnotationPair implements IAnnotationPair {
  private final Annotation annotation;
  private final Object annotationContainer;
  private final Field annotationTargetField;
  private final Method annotationTargetMethod;

  public AnnotationPair(
      Annotation annotation, Object annotationContainer, Field annotationTargetField) {
    this.annotation = annotation;
    this.annotationContainer = annotationContainer;
    this.annotationTargetField = annotationTargetField;
    this.annotationTargetMethod = null;
  }

  @Override
  public Annotation getAnnotation() {
    return annotation;
  }

  @Override
  public Object getAnnotationContainer() {
    return annotationContainer;
  }

  @Override
  public Field getAnnotationTargetField() {
    return annotationTargetField;
  }
}
