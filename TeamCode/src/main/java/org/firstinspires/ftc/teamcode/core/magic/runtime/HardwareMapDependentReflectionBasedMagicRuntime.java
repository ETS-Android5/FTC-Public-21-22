package org.firstinspires.ftc.teamcode.core.magic.runtime;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface HardwareMapDependentReflectionBasedMagicRuntime extends ReflectionBasedMagicRuntime {
    public HardwareMap getHardwareMap();
    public void setHardwareMap(HardwareMap hardwareMap);
}
