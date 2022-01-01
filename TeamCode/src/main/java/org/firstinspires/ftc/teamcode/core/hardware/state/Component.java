package org.firstinspires.ftc.teamcode.core.hardware.state;

import org.firstinspires.ftc.teamcode.core.Namable;

import java.util.List;

public interface Component extends Namable {
  List<? super State> getNextState();
}
