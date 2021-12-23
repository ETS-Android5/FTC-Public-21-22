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

public class SingleServoGripper implements ISingleServoGripper {
  public static final int GRIPPER_ACTION_DELAY_MS = 150;
  private static final String SINGLE_SERVO_GRIPPER_SERVO_NAME = "SINGLE_SERVO_GRIPPER_SERVO";
  private static final double OPEN_POSITION = 0.54;
  private static final double CLOSED_ON_FREIGHT_POSITION = 0.69;
  private static final double CLOSED_ON_TEAM_MARKER_POSITION = 0.73;

  @Hardware(name = SINGLE_SERVO_GRIPPER_SERVO_NAME)
  public Servo singleServoGripperServo;

  private IServoState singleServoGripperServoState;

  public SingleServoGripper() {
    initialize();
  }

  private void initialize() {
    singleServoGripperServoState =
        new ServoState(SINGLE_SERVO_GRIPPER_SERVO_NAME, Direction.FORWARD, OPEN_POSITION);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    return Collections.singletonList(singleServoGripperServoState);
  }

  @Override
  @Observable(key = "SINGLE_SERVO_GRIPPER")
  public SingleServoGripperState getState() {
    return singleServoGripperServoState.getPosition() == CLOSED_ON_FREIGHT_POSITION
        ? SingleServoGripperState.CLOSED_ON_FREIGHT_POSITION
        : singleServoGripperServoState.getPosition() == CLOSED_ON_TEAM_MARKER_POSITION
            ? SingleServoGripperState.CLOSED_ON_TEAM_MARKER_POSITION
            : SingleServoGripperState.OPEN_POSITION;
  }

  @Override
  public synchronized void open() {
    singleServoGripperServoState = singleServoGripperServoState.withPosition(OPEN_POSITION);
  }

  @Override
  public synchronized void close() {
    singleServoGripperServoState =
        singleServoGripperServoState.withPosition(CLOSED_ON_FREIGHT_POSITION);
  }

  @Override
  public synchronized void grabTeamMarker() {
    singleServoGripperServoState =
        singleServoGripperServoState.withPosition(CLOSED_ON_TEAM_MARKER_POSITION);
  }

  @Override
  public synchronized void toggle() {
    singleServoGripperServoState =
        singleServoGripperServoState.withPosition(
            getState() == SingleServoGripperState.CLOSED_ON_FREIGHT_POSITION
                    || getState() == SingleServoGripperState.CLOSED_ON_TEAM_MARKER_POSITION
                ? OPEN_POSITION
                : CLOSED_ON_FREIGHT_POSITION);
  }
}
