package org.firstinspires.ftc.teamcode.core.magic.runtime;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface HardwareMapDependentReflectionBasedMagicRuntime
    extends ReflectionBasedMagicRuntime {

  void setHardwareMap(HardwareMap hardwareMap);

  void lateInit(String deviceName);
}
