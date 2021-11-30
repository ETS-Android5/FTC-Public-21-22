package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.detection.distance.VL53L1X;

@Autonomous(name="SensorTest")
public class SensorTest extends LinearOpMode {
    VL53L1X distanceSensor;

    @Override
    public void runOpMode() {
        try {
            distanceSensor = hardwareMap.get(VL53L1X.class, "VL53L1X");
            waitForStart();
            distanceSensor.initialize();
            distanceSensor.setTimingBudget((short) 20);
            distanceSensor.getDistance(DistanceUnit.MM);
            distanceSensor.onShutdown();
        } catch (Exception ignored) {}
    }
}
