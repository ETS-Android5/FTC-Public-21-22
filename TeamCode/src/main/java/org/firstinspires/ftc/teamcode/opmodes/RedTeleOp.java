package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Alliance;

@TeleOp(name = "Red")
public class RedTeleOp extends TurretBotTeleOpBase {
  public RedTeleOp() {
    super(Alliance.RED);
  }
}
