package org.firstinspires.ftc.teamcode.hardware.robots.turretbot;

import android.util.Log;

import com.google.common.util.concurrent.AtomicDouble;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorPositionReachedCallback;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.detection.distance.InterpolatableUltrasonic;
import org.firstinspires.ftc.teamcode.hardware.detection.gyro.InterpolatableRevGyro;
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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class TurretBot implements Component {
  public static final int AUTO_FIRST_JOINT_OFFSET = 95;
  public static final int TELE_FIRST_JOINT_OFFSET = 230;
  private static final int ELBOW_JOINT_MOVEMENT_DELAY_MS = 250;

  private static final int liftTolerance = 8;
  private static final int nonIntakeLiftTolerance = 24;
  private static final int turretTolerance = 6;
  private static final int nonIntakeTurretTolerance = 18;

  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IIntake<IntakeState> intake = new Intake();
  public final ICarouselSpinner carouselSpinner;
  public final ITurret turret = new Turret();
  public final IDualJointAngularLift lift;
  public final ISingleServoGripper gripper = new SingleServoGripper();
  public final AtomicDouble turretAdjustment = new AtomicDouble(0);
  public final AtomicDouble firstJointAdjustment = new AtomicDouble(0);

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final FtcCamera webcam = new Webcam();

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final InterpolatableRevGyro gyro = new InterpolatableRevGyro(1, 20000000, 3);

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final InterpolatableUltrasonic frontRange =
      new InterpolatableUltrasonic(14, 4, "CB_AUTO_FRONT_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final InterpolatableUltrasonic leftRange =
      new InterpolatableUltrasonic(14, 4, "CB_AUTO_LEFT_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final InterpolatableUltrasonic rightRange =
      new InterpolatableUltrasonic(14, 4, "CB_AUTO_RIGHT_RANGE");

  @SuppressWarnings("unused")
  @AutonomousOnly
  public final InterpolatableUltrasonic rearRange =
      new InterpolatableUltrasonic(14, 4, "CB_AUTO_REAR_RANGE");

  private final int secondJointSafeMovementLvl;
  private final boolean isForAutonomous;
  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();
  private final List<ScheduledFuture<?>> futures = new LinkedList<>();
  private final AtomicReference<Alliance> alliance = new AtomicReference<>();
  private final AtomicReference<TurretBotPosition> setPosition = new AtomicReference<>();
  private final AtomicBoolean gripperIsNearGround = new AtomicBoolean(false);

  @SuppressWarnings("unused")
  @AutonomousOnly
  @Hardware(name = "CB_AUTO_DISTANCE_POLL_TRIGGER")
  public DigitalChannel ultrasonicPollTrigger;

  public TurretBot(Alliance alliance, int jointOffset, boolean isForAutonomous) {
    this.alliance.set(alliance);
    this.lift = new DualJointAngularLift(jointOffset);
    this.secondJointSafeMovementLvl = 300;
    this.isForAutonomous = isForAutonomous;
    this.carouselSpinner = new CarouselSpinner(isForAutonomous);
  }

  private static int turretTolerance(boolean forIntake) {
    return forIntake ? turretTolerance : nonIntakeTurretTolerance;
  }

  private static int liftTolerance(boolean forIntake) {
    return forIntake ? liftTolerance : nonIntakeLiftTolerance;
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
    setPosition.set(null);
  }

  public TurretBotPosition getSetPosition() {
    return setPosition.get();
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

  private MotorPositionReachedCallback setLiftToPosition(int position, int tolerance) {
    int target = position + (int) Math.round(firstJointAdjustment.get());
    lift.setArmOnePosition(target);
    return new MotorPositionReachedCallback(
        DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
        target + lift.getArmOneOffset(),
        tolerance,
        Math.abs(
                target
                    - MotorTrackerPipe.getInstance()
                        .getPositionOf(DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME))
            <= tolerance);
  }

  private MotorPositionReachedCallback setTurretToPosition(int position, int tolerance) {
    int target = position + (int) Math.round(turretAdjustment.get());
    turret.turnToPosition(target);
    return new MotorPositionReachedCallback(
        Turret.TURRET_MOTOR_NAME,
        target,
        tolerance,
        Math.abs(target - MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME))
            <= tolerance);
  }

  private void ensureTurretIsAt(
      int position,
      int liftJointOneTarget,
      MovementStepStrictness stepStrictness,
      boolean isForIntake,
      Runnable andThen) {
    double currentTurretPosition = turret.getState();
    Log.d("TURRETBOT", "AT ENSURE TURRET");
    Log.d("TURRETBOT", "TARGETING LIFT POSITION: " + liftJointOneTarget);
    Log.d("TURRETBOT", "USING MSS " + stepStrictness.name());
    Log.d("TURRETBOT", "IS FOR INTAKE: " + isForIntake);
    if (Math.abs((position + (int) Math.round(turretAdjustment.get())) - currentTurretPosition)
        <= TurretBot.turretTolerance(isForIntake)) {
      Log.d("TURRETBOT", "ALREADY NEAR POSITION");
      setTurretToPosition(position, TurretBot.turretTolerance(isForIntake));
      Log.d("TURRETBOT", "UPDATED TURRET TARGET");
      if (isForIntake) {
        Log.d("TURRETBOT", "UPDATED LIFT TARGET AND SCHEDULED ANDTHEN");
        setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(true)).andThen(andThen);
      } else {
        Log.d("TURRETBOT", "UPDATED LIFT TARGET AND SIMULTANEOUSLY RAN ANDTHEN");
        setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(false));
        andThen.run();
      }
    } else {
      Log.d("TURRETBOT", "NOT NEAR POSITION");
      int liftJointOneMaximizedTarget =
          Math.max(liftJointOneTarget, isForIntake ? 107 : 0);
      Log.d("TURRETBOT", "MAXIMIZED LIFT TARGET: " + liftJointOneMaximizedTarget);
      setLiftToPosition(liftJointOneMaximizedTarget, TurretBot.liftTolerance(isForIntake))
          .andAfterMotorIsBeyond((int) Math.round(liftJointOneMaximizedTarget + firstJointAdjustment.get()),
              TurretBot.liftTolerance(isForIntake),
              () -> {
                if (stepStrictness == MovementStepStrictness.ORDERED_STRICTLY) {
                  setTurretToPosition(position, TurretBot.turretTolerance(isForIntake))
                      .andThen(
                          () ->
                              setLiftToPosition(
                                      liftJointOneTarget, TurretBot.liftTolerance(isForIntake))
                                  .andThen(andThen));
                  return;
                }
                if (liftJointOneTarget
                        > (stepStrictness == MovementStepStrictness.SECOND_JOINT_SAFE
                            ? secondJointSafeMovementLvl
                            : 0)
                    || Math.abs(
                                MotorTrackerPipe.getInstance()
                                        .getPositionOf(Turret.TURRET_MOTOR_NAME)
                                    - position)
                            / Turret.DEGREES_TO_TICKS
                        < 45) {
                  setTurretToPosition(position, TurretBot.turretTolerance(isForIntake));
                  setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(isForIntake));
                  andThen.run();
                } else {
                  int currentTicks =
                      MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME);
                  int ticksInFortyFiveDegrees = (int) Math.round(Turret.DEGREES_TO_TICKS * 45);
                  if (currentTicks < position) {
                    int safeTicksForLiftMovements = position - ticksInFortyFiveDegrees;
                    setTurretToPosition(position, TurretBot.turretTolerance(isForIntake))
                        .andAfterMotorIsBeyond(
                            safeTicksForLiftMovements + (int) Math.round(turretAdjustment.get()),
                            TurretBot.turretTolerance(isForIntake),
                            () -> {
                              setLiftToPosition(
                                  liftJointOneTarget, TurretBot.liftTolerance(isForIntake));
                              andThen.run();
                            });
                  } else {
                    int safeTicksForLiftMovements = position + ticksInFortyFiveDegrees;
                    setTurretToPosition(position, TurretBot.turretTolerance(isForIntake))
                        .andAfterMotorIsBelow(
                            safeTicksForLiftMovements + (int) Math.round(turretAdjustment.get()),
                            TurretBot.turretTolerance(isForIntake),
                            () -> {
                              setLiftToPosition(
                                  liftJointOneTarget, TurretBot.liftTolerance(isForIntake));
                              andThen.run();
                            });
                  }
                }
              });
    }
  }

  public void goToPosition(TurretBotPosition position, boolean inTippedMode) {
    if (setPosition.get() == position) {
      return;
    }
    setPosition.set(position);
    int turretTarget;
    switch (position) {
      case INTAKE_POSITION:
        turretTarget = Turret.TICKS_FRONT;
        safeAdjustArms(turretTarget, inTippedMode, true);
        ensureTurretIsAt(
            turretTarget,
            -230,
            MovementStepStrictness.ORDERED_STRICTLY,
            true,
            () ->
                afterTimedAction(
                    dropFreight(),
                    () -> {
                      if (!isForAutonomous) {
                        intake.beginIntaking();
                      }
                      gripperIsNearGround.set(false);
                    }));
        break;
      case INTAKE_HOVER_POSITION:
        turretTarget = Turret.TICKS_FRONT;
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            0,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> gripperIsNearGround.set(false));
        break;
      case SHARED_CLOSE_POSITION:
        turretTarget = turretPositionForShared();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            0,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.68);
              gripperIsNearGround.set(true);
            });
        break;
      case SHARED_MIDDLE_POSITION:
        turretTarget = turretPositionForShared();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            -40,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.48);
              gripperIsNearGround.set(true);
            });
        break;
      case SHARED_FAR_POSITION:
        turretTarget = turretPositionForShared();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            0,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.2);
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(-260, TurretBot.liftTolerance(false))
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case TIPPED_CLOSE_POSITION:
        turretTarget = turretPositionForShared();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            150,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.66);
              gripperIsNearGround.set(true);
            });
        break;
      case TIPPED_MIDDLE_POSITION:
        turretTarget = turretPositionForShared();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            15,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.44);
              gripperIsNearGround.set(true);
            });
        break;
      case TIPPED_FAR_POSITION:
        turretTarget = turretPositionForShared();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            0,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.2);
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(-210, TurretBot.liftTolerance(false))
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case ALLIANCE_BOTTOM_POSITION:
        turretTarget = turretPositionForAllianceHub();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            0,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.08);
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(-450, TurretBot.liftTolerance(false))
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case ALLIANCE_MIDDLE_POSITION:
        turretTarget = turretPositionForAllianceHub();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            -90,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.23);
              gripperIsNearGround.set(false);
            });
        break;
      case ALLIANCE_TOP_POSITION:
        Log.d("TURRETBOT", "GOING TO ALLIANCE");
        turretTarget = turretPositionForAllianceHub();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            550,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            false,
            () -> {
              lift.setArmTwoPosition(0.52);
              gripperIsNearGround.set(false);
            });
        break;
      case TEAM_MARKER_GRAB_POSITION:
        turretTarget = turretPositionForTeamMarkerGrab();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            0,
            MovementStepStrictness.ORDERED_LOOSELY,
            false,
            () -> {
              lift.setArmTwoPosition(0.08);
              gripper.open();
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(-560, TurretBot.liftTolerance(false))
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case TEAM_MARKER_DEPOSIT_POSITION:
        turretTarget = turretPositionForTeamMarkerDeposit();
        safeAdjustArms(turretTarget, inTippedMode, false);
        ensureTurretIsAt(
            turretTarget,
            570,
            MovementStepStrictness.ORDERED_LOOSELY,
            false,
            () -> {
              lift.setArmTwoPosition(0.32);
              gripperIsNearGround.set(false);
            });
        break;
    }
  }

  public void syncPosition(boolean inTippedMode) {
    TurretBotPosition position = setPosition.get();
    if (position != null) {
      setPosition.set(null);
      goToPosition(position, inTippedMode);
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

  public void clearFutureEvents() {
    futures.forEach(future -> future.cancel(false));
    futures.clear();
    MotorTrackerPipe.getInstance().clearScheduledCallbacks();
    ExitPipe.getInstance().clearScheduledCallbacks();
  }

  public ScheduledExecutorService getExecutorService() {
    return executorService;
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

  private void safeAdjustArms(
      int nextTurretPosition, boolean inTippedMode, boolean approachingIntake) {
    Log.d("TURRETBOT", "BEGIN SAFE ARM ADJUSTMENT");
    int turretTarget = nextTurretPosition + (int) Math.round(turretAdjustment.get());
    if (gripperIsNearGround.get()) {
      Log.d("TURRETBOT", "GRIPPER WAS NEAR GROUND");
      setLiftToPosition(inTippedMode ? 100 : 0,
              TurretBot.liftTolerance(approachingIntake))
          .andThen(
              () -> {
                Log.d("TURRETBOT", "SAFE LIFT ADJUSTMENT POSITION REACHED");
                double currentDegrees = turret.getState();
                if (Math.abs(currentDegrees - (turretTarget / Turret.DEGREES_TO_TICKS)) > 45 || approachingIntake) {
                  Log.d("TURRETBOT", "ADJUSTING SERVO TO " + DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
                  lift.setArmTwoPosition(DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
                }
              });
    } else {
      Log.d("TURRETBOT", "GRIPPER WAS NOT NEAR GROUND");
      double currentDegrees = turret.getState();
      if (Math.abs(currentDegrees - (turretTarget / Turret.DEGREES_TO_TICKS)) > 45 || approachingIntake) {
        Log.d("TURRETBOT", "ADJUSTING SERVO TO " + DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
        lift.setArmTwoPosition(DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
      }
    }
  }

  // Auto methods

  public void turnToHeading(
      double target, double tolerance, Supplier<Boolean> opModeIsActive, Runnable update) {
    TriFunction<Double, Double, Double, Double> powerCurve =
        PowerCurves.generatePowerCurve(0.25, 1);
    Double currentHeading =
        InterpolatablePipe.getInstance().currentDataPointOf(InterpolatableRevGyro.GYRO_NAME);
    double initialHeading = currentHeading;
    double turnSpeed;
    int leftDirection;
    int rightDirection;
    if (Math.abs(target - currentHeading) > tolerance) {
      while (opModeIsActive.get() && Math.abs(target - currentHeading) > tolerance) {
        currentHeading =
            InterpolatablePipe.getInstance().currentDataPointOf(InterpolatableRevGyro.GYRO_NAME);
        turnSpeed = Math.abs(powerCurve.apply(currentHeading, initialHeading, target));
        leftDirection = currentHeading > target ? 1 : -1;
        rightDirection = currentHeading < target ? 1 : -1;
        drivetrain.setLeftPower(turnSpeed * leftDirection);
        drivetrain.setRightPower(turnSpeed * rightDirection);
        update.run();
      }
    }
    drivetrain.setAllPower(0);
    update.run();
  }

  public void runAtHeadingUntilCondition(
      double targetHeading,
      int adjustmentAggression,
      Direction direction,
      Supplier<Boolean> condition,
      Supplier<Double> basePower,
      Supplier<Boolean> opModeIsActive,
      Runnable update) {
    double directionAdjustment = direction == Direction.FORWARD ? -1 : 1;
    while (opModeIsActive.get() && !condition.get()) {
      double currentHeading =
          InterpolatablePipe.getInstance().currentDataPointOf(InterpolatableRevGyro.GYRO_NAME);
      double turnSpeed = Math.abs(currentHeading - targetHeading) / adjustmentAggression;
      double leftDiff = turnSpeed * (currentHeading > targetHeading ? -1 : 1) * directionAdjustment;
      double rightDiff =
          turnSpeed * (currentHeading < targetHeading ? -1 : 1) * directionAdjustment;
      double basePowerVal = basePower.get();
      drivetrain.setLeftPower((basePowerVal * directionAdjustment) + leftDiff);
      drivetrain.setRightPower((basePowerVal * directionAdjustment) + rightDiff);
      update.run();
    }
    drivetrain.setAllPower(0);
    update.run();
  }
}
