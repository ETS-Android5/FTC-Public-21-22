package org.firstinspires.ftc.teamcode.hardware.detection.distance;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@I2cDeviceType
//@DeviceProperties(xmlTag = "VL53L1X", name = "VL53L1X", description = "A 4m Distance Sensor")
public class VL53L1X extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {
    public VL53L1X(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.STMicroelectronics;
    }

    @Override
    public String getDeviceName() {
        return "STMicroelectronics_VL53L1X_Range_Sensor";
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return 0;
    }
}
