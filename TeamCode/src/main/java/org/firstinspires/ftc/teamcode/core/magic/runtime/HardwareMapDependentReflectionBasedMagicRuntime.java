package org.firstinspires.ftc.teamcode.core.magic.runtime;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface HardwareMapDependentReflectionBasedMagicRuntime
    extends ReflectionBasedMagicRuntime {
  HardwareMap getHardwareMap();

  void setHardwareMap(HardwareMap hardwareMap);
}
