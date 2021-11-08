package org.firstinspires.ftc.teamcode.core.annotations.hardware;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD})
public @interface Hardware {
  String name();

  Direction direction() default Direction.FORWARD;

  RunMode runMode() default RunMode.RUN_WITHOUT_ENCODER;

  ZeroPowerBehavior zpBehavior() default ZeroPowerBehavior.BRAKE;
}
