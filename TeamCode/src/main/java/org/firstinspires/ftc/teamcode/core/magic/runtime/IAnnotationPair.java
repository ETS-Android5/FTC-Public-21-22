package org.firstinspires.ftc.teamcode.core.magic.runtime;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

public interface IAnnotationPair {
  Annotation getAnnotation();

  Object getAnnotationContainer();

  Field getAnnotationTargetField();

  Method getAnnotationTargetMethod();
}
