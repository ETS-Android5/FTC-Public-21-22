package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;

@TeleOp(name = "Blue")
public class BlueTeleOp extends TurretBotTeleOpBase {
  public BlueTeleOp() {
    super(Alliance.BLUE);
  }
}
