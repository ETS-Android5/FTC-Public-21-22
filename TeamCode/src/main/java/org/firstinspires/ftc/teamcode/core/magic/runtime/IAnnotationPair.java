package org.firstinspires.ftc.teamcode.core.magic.runtime;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;

public interface IAnnotationPair {
  Annotation getAnnotation();

  Object getAnnotationContainer();

  Field getAnnotationTargetField();
}
