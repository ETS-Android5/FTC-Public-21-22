package org.firstinspires.ftc.teamcode.hardware.robots.turretbot;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.fn.PowerCurves;
import org.firstinspires.ftc.teamcode.core.fn.TriFunction;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.ExitPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.MotorPositionReachedCallback;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.detection.distance.InterpolatableRev2m;
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
  private static final int FIRST_JOINT_OFFSET = 230;
  private static final int SECOND_JOINT_SAFE_MOVEMENT_LVL = FIRST_JOINT_OFFSET + 300;
  private static final int ELBOW_JOINT_MOVEMENT_DELAY_MS = 250;

  private static final int liftTolerance = 8;
  private static final int turretTolerance = 5;

  public final IMecanumDrivetrain drivetrain = new MecanumDrivetrain();
  public final IIntake<IntakeState> intake = new Intake();
  public final ICarouselSpinner carouselSpinner = new CarouselSpinner();
  public final ITurret turret = new Turret();
  public final IDualJointAngularLift lift = new DualJointAngularLift();
  public final ISingleServoGripper gripper = new SingleServoGripper();

  @SuppressWarnings("unused")
  @AutonomousOnly public final FtcCamera webcam = new Webcam();
  @SuppressWarnings("unused")
  @AutonomousOnly public final InterpolatableRevGyro gyro = new InterpolatableRevGyro(
          48,
          25000000,
          3
  );
  @SuppressWarnings("unused")
  @AutonomousOnly public final InterpolatableRev2m frontRange = new InterpolatableRev2m(
          48,
          25000000,
          3,
          "CB_AUTO_FRONT_RANGE"
  );
  @SuppressWarnings("unused")
  @AutonomousOnly public final InterpolatableRev2m leftRange = new InterpolatableRev2m(
          48,
          25000000,
          3,
          "CB_AUTO_LEFT_RANGE"
  );
  @SuppressWarnings("unused")
  @AutonomousOnly public final InterpolatableRev2m rightRange = new InterpolatableRev2m(
          48,
          25000000,
          3,
          "CB_AUTO_RIGHT_RANGE"
  );
  @SuppressWarnings("unused")
  @AutonomousOnly public final InterpolatableRev2m rearRange = new InterpolatableRev2m(
          48,
          25000000,
          3,
          "CB_AUTO_REAR_RANGE"
  );

  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();
  private final List<ScheduledFuture<?>> futures = new LinkedList<>();
  private final AtomicReference<Alliance> alliance = new AtomicReference<>();

  private final AtomicReference<TurretBotPosition> setPosition = new AtomicReference<>();
  private final AtomicBoolean gripperIsNearGround = new AtomicBoolean(false);

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
    setPosition.set(null);
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

  private MotorPositionReachedCallback setLiftToPosition(int position) {
    lift.setArmOnePosition(position);
    return new MotorPositionReachedCallback(
        DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME,
        position,
            TurretBot.liftTolerance,
        Math.abs(
                position
                    - MotorTrackerPipe.getInstance()
                        .getPositionOf(DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME))
            <= TurretBot.liftTolerance);
  }

  private MotorPositionReachedCallback setTurretToPosition(int position) {
    turret.turnToPosition(position);
    return new MotorPositionReachedCallback(
        Turret.TURRET_MOTOR_NAME,
        position,
            TurretBot.turretTolerance,
        Math.abs(position - MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME))
            <= TurretBot.turretTolerance);
  }

  private void ensureTurretIsAt(
          int position,
          int liftJointOneTarget,
          MovementStepStrictness stepStrictness,
          Runnable andThen) {
    double currentTurretPosition = turret.getState();
    if (Math.abs(position - currentTurretPosition) <= TurretBot.turretTolerance) {
      lift.setArmOnePosition(liftJointOneTarget);
      turret.turnToPosition(position);
      andThen.run();
    } else {
      int liftJointOneMaximizedTarget = Math.max(liftJointOneTarget, FIRST_JOINT_OFFSET);
      setLiftToPosition(liftJointOneMaximizedTarget)
          .andAfterMotorIsBeyond(
              FIRST_JOINT_OFFSET,
                  TurretBot.liftTolerance,
              () -> {
                if (stepStrictness == MovementStepStrictness.ORDERED_STRICTLY) {
                  setTurretToPosition(position)
                      .andThen(
                          () -> setLiftToPosition(liftJointOneTarget).andThen(andThen));
                  return;
                }
                if (liftJointOneTarget
                        > (stepStrictness == MovementStepStrictness.SECOND_JOINT_SAFE
                            ? SECOND_JOINT_SAFE_MOVEMENT_LVL
                            : FIRST_JOINT_OFFSET)
                    || Math.abs(
                                MotorTrackerPipe.getInstance()
                                        .getPositionOf(Turret.TURRET_MOTOR_NAME)
                                    - position)
                            / Turret.DEGREES_TO_TICKS
                        < 45) {
                  setTurretToPosition(position);
                  setLiftToPosition(liftJointOneTarget);
                  andThen.run();
                } else {
                  int currentTicks =
                      MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME);
                  int ticksInFortyFiveDegrees = (int) Math.round(Turret.DEGREES_TO_TICKS * 45);
                  if (currentTicks < position) {
                    int safeTicksForLiftMovements = position - ticksInFortyFiveDegrees;
                    setTurretToPosition(position)
                        .andAfterMotorIsBeyond(
                            safeTicksForLiftMovements,
                                TurretBot.turretTolerance,
                            () -> {
                              setLiftToPosition(liftJointOneTarget);
                              andThen.run();
                            });
                  } else {
                    int safeTicksForLiftMovements = position + ticksInFortyFiveDegrees;
                    setTurretToPosition(position)
                        .andAfterMotorIsBelow(
                            safeTicksForLiftMovements,
                                TurretBot.turretTolerance,
                            () -> {
                              setLiftToPosition(liftJointOneTarget);
                              andThen.run();
                            });
                  }
                }
              });
    }
  }

  public void goToPosition(TurretBotPosition position) {
    if (setPosition.get() == position) {
      return;
    }
    setPosition.set(position);
    int turretTarget;
    switch (position) {
      case INTAKE_POSITION:
        turretTarget = Turret.TICKS_FRONT;
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                0,
            MovementStepStrictness.ORDERED_STRICTLY,
            () -> {
              gripper.open();
              intake.beginIntaking();
            });
        break;
      case INTAKE_HOVER_POSITION:
        turretTarget = Turret.TICKS_FRONT;
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget, FIRST_JOINT_OFFSET, MovementStepStrictness.ORDERED_STRICTLY, () -> {});
        break;
      case SHARED_CLOSE_POSITION:
        turretTarget = turretPositionForShared();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET + 69,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> lift.setArmTwoPosition(0.63));
        break;
      case SHARED_MIDDLE_POSITION:
        turretTarget = turretPositionForShared();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET + 10,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> lift.setArmTwoPosition(0.39));
        break;
      case SHARED_FAR_POSITION:
        turretTarget = turretPositionForShared();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> {
              lift.setArmTwoPosition(0.13);
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(FIRST_JOINT_OFFSET - 230)
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case TIPPED_CLOSE_POSITION:
        turretTarget = turretPositionForShared();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET + 88,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> lift.setArmTwoPosition(0.63));
        break;
      case TIPPED_MIDDLE_POSITION:
        turretTarget = turretPositionForShared();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET + 45,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> lift.setArmTwoPosition(0.44));
        break;
      case TIPPED_FAR_POSITION:
        turretTarget = turretPositionForShared();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> {
              lift.setArmTwoPosition(0.13);
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(FIRST_JOINT_OFFSET - 190)
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case ALLIANCE_BOTTOM_POSITION:
        turretTarget = turretPositionForAllianceHub();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> {
              lift.setArmTwoPosition(0);
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () ->
                      setLiftToPosition(FIRST_JOINT_OFFSET - 390)
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case ALLIANCE_MIDDLE_POSITION:
        turretTarget = turretPositionForAllianceHub();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET - 22,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> lift.setArmTwoPosition(0.13));
        break;
      case ALLIANCE_TOP_POSITION:
        turretTarget = turretPositionForAllianceHub();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET + 550,
            MovementStepStrictness.SECOND_JOINT_SAFE,
            () -> lift.setArmTwoPosition(0.43));
        break;
      case TEAM_MARKER_GRAB_POSITION:
        turretTarget = turretPositionForTeamMarkerGrab();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET,
            MovementStepStrictness.ORDERED_LOOSELY,
            () -> {
              lift.setArmTwoPosition(0);
              gripper.open();
              afterTimedAction(
                  ELBOW_JOINT_MOVEMENT_DELAY_MS,
                  () -> setLiftToPosition(FIRST_JOINT_OFFSET - 520)
                          .andThen(() -> gripperIsNearGround.set(true)));
            });
        break;
      case TEAM_MARKER_DEPOSIT_POSITION:
        turretTarget = turretPositionForTeamMarkerDeposit();
        adjustSecondJointForTurretMovement(turretTarget);
        ensureTurretIsAt(
            turretTarget,
                FIRST_JOINT_OFFSET + 570,
            MovementStepStrictness.ORDERED_LOOSELY,
            () -> lift.setArmTwoPosition(0.3));
        break;
    }
  }

  public void goToSharedPosition(double[] position, Runnable afterReached) {
    setPosition.set(null);
    int turretTarget = turretPositionForShared();
    adjustSecondJointForTurretMovement(turretTarget);
    if (position[0] < 150 || position[1] < 0.3) {
      // Handle low movements
      ensureTurretIsAt(
          turretTarget,
              FIRST_JOINT_OFFSET,
          MovementStepStrictness.SECOND_JOINT_SAFE,
          () -> {
            lift.setArmTwoPosition(position[1]);
            afterTimedAction(
                ELBOW_JOINT_MOVEMENT_DELAY_MS,
                () ->
                    setLiftToPosition(FIRST_JOINT_OFFSET + (int) position[0])
                        .andThen(
                            () -> {
                              gripperIsNearGround.set(true);
                              afterReached.run();
                            }));
          });
    } else {
      // Handle closer movements
      ensureTurretIsAt(
          turretTarget,
              FIRST_JOINT_OFFSET + (int) position[0],
          MovementStepStrictness.SECOND_JOINT_SAFE,
          () -> {
            lift.setArmTwoPosition(position[1]);
            afterTimedAction(ELBOW_JOINT_MOVEMENT_DELAY_MS, afterReached);
          });
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

  public void onTrim() {
    // After a manual trim, the robot turret/lift positions are in an unknown state
    // So we mark our setPosition as null
    setPosition.set(null);
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

  private void adjustSecondJointForTurretMovement(int nextTurretPosition) {
    double currentDegrees = turret.getState();
    if (Math.abs(currentDegrees - (nextTurretPosition / Turret.DEGREES_TO_TICKS)) > 45) {
      lift.setArmTwoPosition(0.47);
    }
  }

  // Auto methods

  public void runAtHeadingUntilCondition(
          int target,
          int tolerance,
          Direction direction,
          Supplier<Boolean> condition,
          Supplier<Double> basePower,
          Supplier<Boolean> opModeIsActive,
          Runnable update) {
    TriFunction<Integer, Integer, Integer, Double> powerCurve =
            PowerCurves.generatePowerCurve(0.25, 2.33);
    Double currentHeading = InterpolatablePipe.getInstance().currentDataPointOf(InterpolatableRevGyro.GYRO_NAME);
    double initialHeading = currentHeading;
    Double turnSpeed;
    int leftDirection;
    int rightDirection;
    if (Math.abs(target - currentHeading) > tolerance) {
      while (opModeIsActive.get() && Math.abs(target - currentHeading) > tolerance) {
        currentHeading = InterpolatablePipe.getInstance().currentDataPointOf(InterpolatableRevGyro.GYRO_NAME);
        turnSpeed =
                powerCurve.apply(
                        (int) Math.round(currentHeading), (int) Math.round(initialHeading), target);
        leftDirection = currentHeading > target ? -1 : 1;
        rightDirection = currentHeading < target ? -1 : 1;
        drivetrain.setLeftPower(turnSpeed * leftDirection);
        drivetrain.setRightPower(turnSpeed * rightDirection);
        update.run();
      }
    }
    drivetrain.setAllPower(0);
    update.run();
    double leftDiff;
    double rightDiff;
    double directionAdjustment = direction == Direction.FORWARD ? -1 : 1;
    while (opModeIsActive.get() && !condition.get()) {
      currentHeading = InterpolatablePipe.getInstance().currentDataPointOf(InterpolatableRevGyro.GYRO_NAME);
      turnSpeed =
              powerCurve.apply(
                      (int) Math.round(currentHeading), (int) Math.round(initialHeading), target);
      leftDiff = turnSpeed * (currentHeading > target ? -1 : 1) * directionAdjustment;
      rightDiff = turnSpeed * (currentHeading < target ? -1 : 1) * directionAdjustment;
      double basePowerVal = basePower.get();
      drivetrain.setLeftPower((basePowerVal * directionAdjustment) + leftDiff);
      drivetrain.setRightPower((basePowerVal * directionAdjustment) + rightDiff);
      update.run();
    }
    drivetrain.setAllPower(0);
    update.run();
  }
}
