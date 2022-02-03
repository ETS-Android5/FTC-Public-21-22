package org.firstinspires.ftc.teamcode.hardware.robots.turretbot;

import com.google.common.util.concurrent.AtomicDouble;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
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
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ITapeMeasure;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.ITurret;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.SingleServoGripper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.SingleServoGripperState;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.TapeMeasure;
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
  public static final int AUTO_FIRST_JOINT_OFFSET = 115;
  public static final int TELE_FIRST_JOINT_OFFSET = 260;
  private static final int ELBOW_JOINT_MOVEMENT_DELAY_MS = 250;

  private static final int liftTolerance = 8;
  private static final int nonIntakeLiftTolerance = 24;
  private static final int turretTolerance = 6;
  private static final int nonIntakeTurretTolerance = 18;

  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IIntake<IntakeState> intake = new Intake();
  public final ICarouselSpinner carouselSpinner;
  public final ITapeMeasure tapeMeasure = new TapeMeasure();
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
                (target + lift.getArmOneOffset())
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
      int position, int liftJointOneTarget, boolean forIntake, Runnable andThen) {
    double currentTurretPosition = turret.getState();
    if (Math.abs((position + (int) Math.round(turretAdjustment.get())) - currentTurretPosition)
        <= TurretBot.turretTolerance(forIntake)) {
      setTurretToPosition(position, TurretBot.turretTolerance(forIntake));
      if (forIntake) {
        setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(true)).andThen(andThen);
      } else {
        setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(false));
        andThen.run();
      }
    } else {
      int liftJointOneMaximizedTarget = Math.max(liftJointOneTarget, forIntake ? 107 : 0);
      setLiftToPosition(liftJointOneMaximizedTarget, TurretBot.liftTolerance(forIntake))
          .andAfterMotorIsBeyond(
              (int)
                  Math.round(
                      liftJointOneMaximizedTarget
                          + lift.getArmOneOffset()
                          + firstJointAdjustment.get()),
              TurretBot.liftTolerance(forIntake),
              () -> completeTurretMovement(forIntake, position, liftJointOneTarget, andThen));
    }
  }

  private void completeTurretMovement(
      boolean forIntake, int position, int liftJointOneTarget, Runnable andThen) {
    if (forIntake) {
      setTurretToPosition(position, TurretBot.turretTolerance(true))
          .andThen(
              () ->
                  setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(true))
                      .andThen(andThen));
    } else if (liftJointOneTarget > secondJointSafeMovementLvl
        || Math.abs(
                    MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME)
                        - position)
                / Turret.DEGREES_TO_TICKS
            < 45) {
      setTurretToPosition(position, TurretBot.turretTolerance(false));
      setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(false));
      andThen.run();
    } else {
      int currentTicks = MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME);
      int ticksInFortyFiveDegrees = (int) Math.round(Turret.DEGREES_TO_TICKS * 45);
      if (currentTicks < position) {
        int safeTicksForLiftMovements = position - ticksInFortyFiveDegrees;
        setTurretToPosition(position, TurretBot.turretTolerance(false))
            .andAfterMotorIsBeyond(
                safeTicksForLiftMovements + (int) Math.round(turretAdjustment.get()),
                TurretBot.turretTolerance(false),
                () -> {
                  setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(false));
                  andThen.run();
                });
      } else {
        int safeTicksForLiftMovements = position + ticksInFortyFiveDegrees;
        setTurretToPosition(position, TurretBot.turretTolerance(false))
            .andAfterMotorIsBelow(
                safeTicksForLiftMovements + (int) Math.round(turretAdjustment.get()),
                TurretBot.turretTolerance(false),
                () -> {
                  setLiftToPosition(liftJointOneTarget, TurretBot.liftTolerance(false));
                  andThen.run();
                });
      }
    }
  }

  public void goToPosition(TurretBotPosition position, boolean inTippedMode) {
    if (setPosition.get() == position) {
      return;
    }
    setPosition.set(position);
    switch (position) {
      case INTAKE_POSITION:
        goToIntakePosition(inTippedMode);
        return;
      case INTAKE_HOVER_POSITION:
        goToIntakeHoverPosition(inTippedMode);
        return;
      case SHARED_CLOSE_POSITION:
        goToSharedClosePosition(inTippedMode);
        return;
      case SHARED_MIDDLE_POSITION:
        goToSharedMiddlePosition(inTippedMode);
        return;
      case SHARED_FAR_POSITION:
        goToSharedFarPosition(inTippedMode);
        return;
      case TIPPED_CLOSE_POSITION:
        goToTippedClosePosition(inTippedMode);
        return;
      case TIPPED_MIDDLE_POSITION:
        goToTippedMiddlePosition(inTippedMode);
        return;
      case TIPPED_FAR_POSITION:
        goToTippedFarPosition(inTippedMode);
        return;
      case ALLIANCE_BOTTOM_POSITION:
        goToAllianceBottomPosition(inTippedMode);
        return;
      case ALLIANCE_MIDDLE_POSITION:
        goToAllianceMiddlePosition(inTippedMode);
        return;
      case ALLIANCE_TOP_POSITION:
        goToAllianceTopPosition(inTippedMode);
        return;
      case TEAM_MARKER_GRAB_POSITION:
        goToTeamMarkerGrabPosition(inTippedMode);
        return;
      case TEAM_MARKER_DEPOSIT_POSITION:
        goToTeamMarkerDepositPosition();
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
    }
    return 0;
  }

  public int grabFreight() {
    if (gripper.getState() != SingleServoGripperState.CLOSED_ON_FREIGHT_POSITION) {
      gripper.close();
      return SingleServoGripper.GRIPPER_ACTION_DELAY_MS;
    }
    return 0;
  }

  public int grabTeamMarker() {
    if (gripper.getState() != SingleServoGripperState.CLOSED_ON_TEAM_MARKER_POSITION) {
      gripper.grabTeamMarker();
      return SingleServoGripper.GRIPPER_ACTION_DELAY_MS;
    }
    return 0;
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

  private void goToIntakePosition(boolean inTippedMode) {
    int turretTarget = Turret.TICKS_FRONT;
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, true),
        () ->
            ensureTurretIsAt(
                turretTarget,
                -230,
                true,
                () ->
                    afterTimedAction(
                        dropFreight(),
                        () -> {
                          if (!isForAutonomous) {
                            intake.beginIntaking();
                          }
                          gripperIsNearGround.set(false);
                        })));
  }

  private void goToIntakeHoverPosition(boolean inTippedMode) {
    int turretTarget = Turret.TICKS_FRONT;
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () -> ensureTurretIsAt(turretTarget, 0, false, () -> gripperIsNearGround.set(false)));
  }

  private void goToSharedClosePosition(boolean inTippedMode) {
    int turretTarget = turretPositionForShared();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                0,
                false,
                () -> {
                  lift.setArmTwoPosition(0.68);
                  gripperIsNearGround.set(false);
                }));
  }

  private void goToSharedMiddlePosition(boolean inTippedMode) {
    int turretTarget = turretPositionForShared();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                -40,
                false,
                () -> {
                  lift.setArmTwoPosition(0.48);
                  gripperIsNearGround.set(false);
                }));
  }

  private void goToSharedFarPosition(boolean inTippedMode) {
    int turretTarget = turretPositionForShared();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                0,
                false,
                () -> {
                  lift.setArmTwoPosition(0.2);
                  afterTimedAction(
                      ELBOW_JOINT_MOVEMENT_DELAY_MS,
                      () ->
                          setLiftToPosition(-260, TurretBot.liftTolerance(false))
                              .andThen(() -> gripperIsNearGround.set(true)));
                }));
  }

  private void goToTippedClosePosition(boolean inTippedMode) {
    int turretTarget = turretPositionForShared();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                150,
                false,
                () -> {
                  lift.setArmTwoPosition(0.66);
                  gripperIsNearGround.set(true);
                }));
  }

  private void goToTippedMiddlePosition(boolean inTippedMode) {
    int turretTarget = turretPositionForShared();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                15,
                false,
                () -> {
                  lift.setArmTwoPosition(0.44);
                  gripperIsNearGround.set(true);
                }));
  }

  private void goToTippedFarPosition(boolean inTippedMode) {
    int turretTarget = turretPositionForShared();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                -20,
                false,
                () -> {
                  lift.setArmTwoPosition(0.2);
                  afterTimedAction(
                      ELBOW_JOINT_MOVEMENT_DELAY_MS,
                      () ->
                          setLiftToPosition(-210, TurretBot.liftTolerance(false))
                              .andThen(() -> gripperIsNearGround.set(true)));
                }));
  }

  private void goToAllianceBottomPosition(boolean inTippedMode) {
    int turretTarget = turretPositionForAllianceHub();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                -20,
                false,
                () -> {
                  lift.setArmTwoPosition(0.08);
                  afterTimedAction(
                      ELBOW_JOINT_MOVEMENT_DELAY_MS,
                      () ->
                          setLiftToPosition(-460, TurretBot.liftTolerance(false))
                              .andThen(() -> gripperIsNearGround.set(true)));
                }));
  }

  private void goToAllianceMiddlePosition(boolean inTippedMode) {
    int turretTarget = turretPositionForAllianceHub();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                -110,
                false,
                () -> {
                  lift.setArmTwoPosition(0.23);
                  gripperIsNearGround.set(false);
                }));
  }

  private void goToAllianceTopPosition(boolean inTippedMode) {
    int turretTarget = turretPositionForAllianceHub();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                530,
                false,
                () -> {
                  lift.setArmTwoPosition(0.52);
                  gripperIsNearGround.set(false);
                }));
  }

  private void goToTeamMarkerGrabPosition(boolean inTippedMode) {
    int turretTarget = turretPositionForTeamMarkerGrab();
    afterTimedAction(
        safeAdjustArms(turretTarget, inTippedMode, false),
        () ->
            ensureTurretIsAt(
                turretTarget,
                0,
                false,
                () -> {
                  lift.setArmTwoPosition(0.08);
                  gripper.open();
                  afterTimedAction(
                      ELBOW_JOINT_MOVEMENT_DELAY_MS,
                      () ->
                          setLiftToPosition(-570, TurretBot.liftTolerance(false))
                              .andThen(() -> gripperIsNearGround.set(true)));
                }));
  }

  private void goToTeamMarkerDepositPosition() {
    int turretTarget = turretPositionForTeamMarkerDeposit();
    ensureTurretIsAt(
        turretTarget,
        570,
        false,
        () -> {
          lift.setArmTwoPosition(0.32);
          gripperIsNearGround.set(false);
        });
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

  private int safeAdjustArms(
      int nextTurretPosition, boolean inTippedMode, boolean approachingIntake) {
    int turretTarget = nextTurretPosition + (int) Math.round(turretAdjustment.get());
    if (gripperIsNearGround.get()) {
      setLiftToPosition(inTippedMode ? 100 : 0, TurretBot.liftTolerance(approachingIntake))
          .andThen(
              () -> {
                double currentDegrees = turret.getState();
                if (Math.abs(currentDegrees - (turretTarget / Turret.DEGREES_TO_TICKS)) > 45
                    || approachingIntake) {
                  lift.setArmTwoPosition(DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
                }
              });
    } else {
      double currentDegrees = turret.getState();
      if (Math.abs(currentDegrees - (turretTarget / Turret.DEGREES_TO_TICKS)) > 45
          || approachingIntake) {
        lift.setArmTwoPosition(DualJointAngularLift.LIFT_JOINT_TWO_INTAKE_POSITION);
      }
    }
    return SingleServoGripper.GRIPPER_ACTION_DELAY_MS;
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

  public void driveForward(
      double distanceInches, double tolerance, Supplier<Boolean> opModeIsActive, Runnable update) {
    driveForward(0.5, 1, distanceInches, tolerance, opModeIsActive, update);
  }

  public void driveForward(
      double maxPower,
      double rampSlope,
      double distanceInches,
      double tolerance,
      Supplier<Boolean> opModeIsActive,
      Runnable update) {
    TriFunction<Double, Double, Double, Double> powerCurve =
        PowerCurves.generatePowerCurve(maxPower, rampSlope);
    double initEncoderValue = drivetrain.avgEncoderValue();
    while (opModeIsActive.get()
        && Math.abs(drivetrain.avgEncoderValue() - initEncoderValue) > tolerance) {
      drivetrain.setAllPower(
          powerCurve.apply(
              drivetrain.avgEncoderValue(),
              initEncoderValue,
              distanceInches * MecanumDrivetrain.TICKS_PER_INCH));
      update.run();
    }
    drivetrain.setAllPower(0);
    update.run();
  }
}
