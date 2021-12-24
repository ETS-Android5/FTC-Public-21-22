package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorPositionReachedCallback;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.detection.vision.FtcCamera;
import org.firstinspires.ftc.teamcode.hardware.detection.vision.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ICarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ISingleServoGripper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ITurret;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.SingleServoGripper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.SingleServoGripperState;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.IMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.IIntake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.intakes.IntakeState;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.DualJointAngularLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts.IDualJointAngularLift;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class TurretBot implements Component {
  private static final int firstJointOffset = 230;

  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IIntake<IntakeState> intake = new Intake();
  public final ICarouselSpinner carouselSpinner = new CarouselSpinner();
  public final ITurret turret = new Turret();
  public final IDualJointAngularLift lift = new DualJointAngularLift();
  public final ISingleServoGripper gripper = new SingleServoGripper();

  @AutonomousOnly public final FtcCamera webcam = new Webcam();
  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();
  private final List<ScheduledFuture<?>> futures = new LinkedList<>();

  private final AtomicReference<Alliance> alliance = new AtomicReference<>();

  public TurretBot(Alliance alliance) {
    assert alliance != null;
    this.alliance.set(alliance);
  }

  @Override
  public String getName() {
    return this.getClass().getName();
  }

  @Override
  public List<? super State> getNextState() {
    List<? super State> states = new LinkedList<>();
    Arrays.asList(drivetrain, intake, carouselSpinner, turret, lift, gripper)
        .forEach((component) -> component.getNextState().forEach((s) -> states.add((State) s)));
    return states;
  }

  public Alliance getAlliance() {
    return alliance.get();
  }

  public void setAlliance(Alliance alliance) {
    this.alliance.set(alliance);
  }

  public void afterTimedAction(int ms, Runnable r) {
    if (ms > 0) {
      futures.add(
          executorService.schedule(
              () -> ExitPipe.getInstance().onNextTick(r), ms, TimeUnit.MILLISECONDS));
    } else {
      r.run();
    }
  }

  public MotorPositionReachedCallback setLiftToPosition(int position, int tolerance) {
    lift.setArmOnePosition(position);
    return new MotorPositionReachedCallback(
        DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
        position,
        tolerance,
        Math.abs(
                position
                    - MotorTrackerPipe.getInstance()
                        .getPositionOf(DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME))
            <= tolerance);
  }

  public MotorPositionReachedCallback setTurretToPosition(int position, int tolerance) {
    turret.turnToPosition(position);
    return new MotorPositionReachedCallback(
        Turret.TURRET_MOTOR_NAME,
        position,
        tolerance,
        Math.abs(position - MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME))
            <= tolerance);
  }

  public void ensureTurretIsAt(
      int position, int tolerance, int liftJointOneTarget, Runnable andThen) {
    double currentTurretPosition = turret.getState();
    if (Math.abs(position - currentTurretPosition) <= tolerance) {
      lift.setArmOnePosition(liftJointOneTarget);
      turret.turnToPosition(position);
      andThen.run();
    } else {
      int liftJointOneMaximizedTarget = Math.max(liftJointOneTarget, firstJointOffset);
      setLiftToPosition(liftJointOneMaximizedTarget, tolerance)
          .andThen(
              () ->
                  setTurretToPosition(position, tolerance)
                      .andThen(
                          () -> setLiftToPosition(liftJointOneTarget, tolerance).andThen(andThen)));
    }
  }

  public void goToPosition(TurretBotPosition position) {
    lift.setArmTwoPosition(0.47);
    switch (position) {
      case INTAKE_POSITION:
        ensureTurretIsAt(
            Turret.TICKS_FRONT,
            5,
            0,
            () -> {
              gripper.open();
              intake.beginIntaking();
            });
        break;
      case SHARED_CLOSE_POSITION:
        ensureTurretIsAt(
            turretPositionForShared(),
            5,
            firstJointOffset + 69,
            () -> lift.setArmTwoPosition(0.56));
        break;
      case SHARED_MIDDLE_POSITION:
        ensureTurretIsAt(
                turretPositionForShared(),
                5,
                firstJointOffset + 10,
                () -> lift.setArmTwoPosition(0.39));
        break;
      case SHARED_FAR_POSITION:
        ensureTurretIsAt(
                turretPositionForShared(),
                5,
                firstJointOffset - 230,
                () -> lift.setArmTwoPosition(0.13));
        break;
      case TIPPED_CLOSE_POSITION:
        // TODO: VALUES
        break;
      case TIPPED_MIDDLE_POSITION:
        // TODO: VALUES
        break;
      case TIPPED_FAR_POSITION:
        // TODO: VALUES
        break;
      case ALLIANCE_BOTTOM_POSITION:
        ensureTurretIsAt(
            turretPositionForAllianceHub(),
            5,
            firstJointOffset - 390,
            () -> lift.setArmTwoPosition(0));
        break;
      case ALLIANCE_MIDDLE_POSITION:
        ensureTurretIsAt(
            turretPositionForAllianceHub(),
            5,
            firstJointOffset - 22,
            () -> lift.setArmTwoPosition(0.13));
        break;
      case ALLIANCE_TOP_POSITION:
        ensureTurretIsAt(
            turretPositionForAllianceHub(),
            5,
            firstJointOffset + 550,
            () -> lift.setArmTwoPosition(0.43));
        break;
      case TEAM_MARKER_GRAB_POSITION:
        ensureTurretIsAt(
            turretPositionForTeamMarkerGrab(),
            5,
            firstJointOffset - 540,
            () -> {
              lift.setArmTwoPosition(0);
              gripper.open();
            });
        break;
      case TEAM_MARKER_DEPOSIT_POSITION:
        ensureTurretIsAt(
            turretPositionForTeamMarkerDeposit(),
            5,
            firstJointOffset + 570,
            () -> lift.setArmTwoPosition(0.3));
        break;
    }
  }

  public int dropFreight() {
    if (gripper.getState() != SingleServoGripperState.OPEN_POSITION) {
      gripper.open();
      return SingleServoGripper.GRIPPER_ACTION_DELAY_MS;
    } else {
      return 0;
    }
  }

  public int grabFreight() {
    if (gripper.getState() != SingleServoGripperState.CLOSED_ON_FREIGHT_POSITION) {
      gripper.close();
      return SingleServoGripper.GRIPPER_ACTION_DELAY_MS;
    } else {
      return 0;
    }
  }

  public int grabTeamMarker() {
    if (gripper.getState() != SingleServoGripperState.CLOSED_ON_TEAM_MARKER_POSITION) {
      gripper.grabTeamMarker();
      return SingleServoGripper.GRIPPER_ACTION_DELAY_MS;
    } else {
      return 0;
    }
  }

  public void outtakeIfNecessary() {
    if (intake.getState() == IntakeState.INTAKING) {
      intake.beginOuttakingSlowly();
      afterTimedAction(1750, intake::stop);
    }
  }

  public void clearFutureEvents() {
    futures.forEach(future -> future.cancel(false));
    futures.clear();
    MotorTrackerPipe.getInstance().clearScheduledCallbacks();
    ExitPipe.getInstance().clearScheduledCallbacks();
  }

  private int turretPositionForShared() {
    switch (alliance.get()) {
      case RED:
        return Turret.TICKS_RIGHT;
      case BLUE:
        return Turret.TICKS_LEFT;
    }
    return 0;
  }

  private int turretPositionForAllianceHub() {
    switch (alliance.get()) {
      case RED:
        return Turret.TICKS_CCW_BACK;
      case BLUE:
        return Turret.TICKS_CW_BACK;
    }
    return 0;
  }

  private int turretPositionForTeamMarkerGrab() {
    switch (alliance.get()) {
      case RED:
        return Turret.TICKS_LEFT;
      case BLUE:
        return Turret.TICKS_RIGHT;
    }
    return 0;
  }

  private int turretPositionForTeamMarkerDeposit() {
    switch (alliance.get()) {
      case RED:
        return Turret.TICKS_CCW_BACK;
      case BLUE:
        return Turret.TICKS_CW_BACK;
    }
    return 0;
  }
}
