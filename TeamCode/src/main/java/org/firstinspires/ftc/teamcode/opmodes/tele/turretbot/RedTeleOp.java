package org.firstinspires.ftc.teamcode.opmodes.tele.turretbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.game.related.Alliance;

@TeleOp(name = "Red")
@SuppressWarnings("unused")
public class RedTeleOp extends TurretBotTeleOpBase {
  public RedTeleOp() {
    super(Alliance.RED);
  }
}
