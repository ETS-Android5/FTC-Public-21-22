package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.annotations.Observable;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.hardware.state.IServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.ServoState;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;

import java.util.Collections;
import java.util.List;

public class OuttakeBucket implements IOuttakeBucket {
  private static final String OUTTAKE_BUCKET_SERVO_NAME = "OUTTAKE_BUCKET_SERVO";

  private static final double CARRY_POSITION = .3;
  private static final double DUMP_POSITION = .9;

  @Hardware(name = OUTTAKE_BUCKET_SERVO_NAME)
  public Servo outtakeServo;

  private IServoState outtakeServoState;

  public OuttakeBucket() {
    initialize();
  }

  private void initialize() {
    outtakeServoState =
        new ServoState(OUTTAKE_BUCKET_SERVO_NAME, Direction.FORWARD, CARRY_POSITION);
  }

  @Override
  public synchronized void dump() {
    outtakeServoState = outtakeServoState.withPosition(DUMP_POSITION);
  }

  @Override
  public synchronized void carry() {
    outtakeServoState = outtakeServoState.withPosition(CARRY_POSITION);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Collections.singletonList(outtakeServoState);
  }

  @Override
  @Observable(key = "OUTTAKE_BUCKET")
  public OuttakeBucketState getState() {
    return outtakeServoState.getPosition() == CARRY_POSITION
        ? OuttakeBucketState.CARRY_POSITION
        : OuttakeBucketState.DUMP_POSITION;
  }
}
