package org.firstinspires.ftc.teamcode.core.game.related;

public enum Alliance {
  BLUE,
  RED;

  public Alliance opposite() {
    switch (this) {
      case BLUE:
        return RED;
      case RED:
        return BLUE;
    }
    return null;
  }
}
