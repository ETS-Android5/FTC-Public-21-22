package org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;

public interface IAWDClassicDrivetrain extends Component {
  void setAllPower(double power);

  void setLeftPower(double power);

  void setRightPower(double power);

  void setAllTarget(int ticks);

  void setLeftTarget(int ticks);

  void setRightTarget(int ticks);

  void reset();

  void setRunMode(RunMode runMode);
}
