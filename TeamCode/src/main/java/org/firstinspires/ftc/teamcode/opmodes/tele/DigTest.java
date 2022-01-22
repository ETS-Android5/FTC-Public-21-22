package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "DigTest")
@Disabled
public class DigTest extends OpMode {
  private static final double VOLTS_TO_INCHES_CONVERSION = (1 / 0.0049);

  private DigitalChannel chan0;

  private AnalogInput an0;
  private AnalogInput an1;
  private AnalogInput an2;
  private AnalogInput an3;
  private long lastMeasurement = 0;

  @Override
  public void init() {
    chan0 = hardwareMap.get(DigitalChannel.class, "Dig0");
    an0 = hardwareMap.get(AnalogInput.class, "An0");
    an1 = hardwareMap.get(AnalogInput.class, "An1");
    an2 = hardwareMap.get(AnalogInput.class, "An2");
    an3 = hardwareMap.get(AnalogInput.class, "An3");
    initSensor(chan0);
  }

  @Override
  public void start() {}

  @Override
  public void loop() {
    if (System.currentTimeMillis() - lastMeasurement > 49) {
      chan0.setState(true);
      long start = System.currentTimeMillis();
      while (System.currentTimeMillis() - start < 1) {}
      chan0.setState(false);
      lastMeasurement = System.currentTimeMillis();
    }
    for (int i = 0; i < 4; i++) {
      telemetry.addData(
          "Distance " + i,
          new AnalogInput[] {an0, an1, an2, an3}[i].getVoltage() * VOLTS_TO_INCHES_CONVERSION);
    }
  }

  private void initSensor(DigitalChannel channel) {
    channel.setMode(DigitalChannel.Mode.OUTPUT);
    channel.setState(false);
  }
}
