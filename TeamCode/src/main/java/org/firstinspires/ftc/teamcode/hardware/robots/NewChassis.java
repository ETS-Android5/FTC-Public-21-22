package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.AutonomousOnly;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.CallbackData;
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

public class NewChassis implements Component {
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
            Math.abs(position - MotorTrackerPipe.getInstance().getPositionOf(DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME)) <= tolerance
    );
  }

  public MotorPositionReachedCallback setTurretToPosition(int position, int tolerance) {
    turret.turnToPosition(position);
    return new MotorPositionReachedCallback(
            Turret.TURRET_MOTOR_NAME,
            position,
            tolerance,
            Math.abs(position - MotorTrackerPipe.getInstance().getPositionOf(DualJointAngularLift.LIFT_JOINT_ONE_MOTOR_NAME)) <= tolerance
    );
  }

  public void ensureTurretIsAt(int position, int tolerance, int liftJointOneTarget, Runnable andThen) {
    double currentTurretPosition = turret.getState();
    if (Math.abs(position - currentTurretPosition) <= tolerance) {
      lift.setArmOnePosition(liftJointOneTarget);
      turret.turnToPosition(position);
      andThen.run();
    } else {
      int liftJointOneMaximizedTarget = Math.max(liftJointOneTarget, firstJointOffset);
      setLiftToPosition(liftJointOneMaximizedTarget, tolerance)
              .andThen(() -> setTurretToPosition(position, tolerance)
                      .andThen(() -> setLiftToPosition(liftJointOneTarget, tolerance)
                              .andThen(andThen)));
    }
  }

  public void goToPosition(NewChassisPosition position) {
    lift.setArmTwoPosition(0.47);
    switch (position) {
      case INTAKE_POSITION:
        ensureTurretIsAt(Turret.TICKS_FRONT, 5, 0, () -> {
          gripper.open();
          intake.beginIntaking();
        });
        break;
      case SHARED_CLOSE_POSITION:
        ensureTurretIsAt(Turret.TICKS_RIGHT, 5, firstJointOffset + 69, () -> {
          lift.setArmTwoPosition(0.56);
        });
        break;
      case SHARED_MIDDLE_POSITION:
        break;
      case SHARED_FAR_POSITION:
        break;
      case TIPPED_CLOSE_POSITION:
        break;
      case TIPPED_MIDDLE_POSITION:
        break;
      case TIPPED_FAR_POSITION:
        break;
      case ALLIANCE_BOTTOM_POSITION:
        break;
      case ALLIANCE_MIDDLE_POSITION:
        break;
      case ALLIANCE_TOP_POSITION:
        break;
      case TEAM_MARKER_GRAB_POSITION:
        break;
      case TEAM_MARKER_DEPOSIT_POSITION:
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

  public void outtakeIfNecessary() {
    if (intake.getState() == IntakeState.INTAKING) {
      intake.beginOuttaking();
      afterTimedAction(750, intake::stop);
    }
  }

  public void clearFutureEvents() {
    futures.forEach(future -> future.cancel(false));
    futures.clear();
    MotorTrackerPipe.getInstance().clearScheduledCallbacks();
    ExitPipe.getInstance().clearScheduledCallbacks();
  }
}
