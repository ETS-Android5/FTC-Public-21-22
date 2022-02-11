package org.firstinspires.ftc.teamcode.opmodes.tele.turretbot;

import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class TurretBotTeleOpBase extends EnhancedTeleOp {
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.002;
  public final AtomicBoolean tapeMeasureMode = new AtomicBoolean(false);
  private final TurretBot robot;
  private final AtomicBoolean halfSpeed = new AtomicBoolean(true);
  private final AtomicBoolean allianceHubMode = new AtomicBoolean(false);
  private final AtomicBoolean tippedMode = new AtomicBoolean(false);
  private final List<ScheduledFuture<?>> futures = new LinkedList<>();
  private boolean firstTime = true;

  public TurretBotTeleOpBase(Alliance alliance) {
    super(new TurretBot(alliance, TurretBot.TELE_FIRST_JOINT_OFFSET, false));
    this.robot = (TurretBot) super.robotObject;
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void initLoop() {}

  @Override
  public void onStartPressed() {
    futures.add(
        robot
            .getExecutorService()
            .schedule(
                () -> {
                  controller1.vibrate(0.25, 0.25, 3000);
                  controller2.vibrate(0.25, 0.25, 3000);
                },
                81,
                TimeUnit.SECONDS));

    futures.add(
        robot
            .getExecutorService()
            .schedule(
                () -> {
                  controller1.vibrate(0.5, 0.5, 3000);
                  controller2.vibrate(0.5, 0.5, 3000);
                },
                84,
                TimeUnit.SECONDS));
    futures.add(
        robot
            .getExecutorService()
            .schedule(
                () -> {
                  controller1.vibrate(1, 1, 3000);
                  controller2.vibrate(1, 1, 3000);
                },
                87,
                TimeUnit.SECONDS));

    controller1.registerOnPressedCallback(
        () -> halfSpeed.set(!halfSpeed.get()), true, BooleanSurface.X);

    controller1.registerOnPressedCallback(
        () -> robot.lift.setArmTwoPosition(robot.lift.getState().second - .001),
        false,
        BooleanSurface.LEFT_BUMPER);
    controller1.registerOnPressedCallback(
        () -> robot.lift.setArmTwoPosition(robot.lift.getState().second + .001),
        false,
        BooleanSurface.RIGHT_BUMPER);

    controller1.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          TurretBotPosition currentPosition = robot.getSetPosition();
          if (firstTime
              || currentPosition == TurretBotPosition.INTAKE_HOVER_POSITION
              || currentPosition == TurretBotPosition.INTAKE_POSITION) {
            robot.goToPosition(TurretBotPosition.INTAKE_POSITION, tippedMode.get());
            firstTime = false;
          } else {
            robot.afterTimedAction(
                robot.dropFreight() + (allianceHubMode.get() ? 750 : 0),
                () -> {
                  robot.grabTeamMarker();
                  robot.goToPosition(TurretBotPosition.INTAKE_POSITION, tippedMode.get());
                });
          }
        },
        true,
        BooleanSurface.A);

    controller1.registerOnPressedCallback(robot.gripper::toggle, true, BooleanSurface.DPAD_RIGHT);

    controller1.registerOnPressedCallback(() -> {
        boolean prev = tapeMeasureMode.get();
        tapeMeasureMode.set(!prev);
        if (prev) {
            robot.tapeMeasure.setLengthRate(0);
        }
    }, true, BooleanSurface.DPAD_DOWN);

    controller2.registerOnPressedCallback(
        () -> {
          firstTime = false;
          robot.clearFutureEvents();
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(TurretBotPosition.INTAKE_HOVER_POSITION, tippedMode.get());
              });
        },
        true,
        BooleanSurface.A);

    controller2.registerOnPressedCallback(
        robot.intake::toggleIntaking, true, BooleanSurface.RIGHT_BUMPER);
    controller2.registerOnPressedCallback(
        robot.intake::toggleOuttaking, true, BooleanSurface.LEFT_BUMPER);

    controller2.registerOnPressedCallback(
        carouselMoveForAlliance(robot.getAlliance()), true, BooleanSurface.LEFT_STICK);

    controller2.registerOnPressedCallback(
        () -> {
          robot.setAlliance(robot.getAlliance().opposite());
          controller2.vibrate(1, 1, 250);
        },
        true,
        BooleanSurface.RIGHT_STICK);

    controller2.registerOnPressedCallback(
        () -> {
          allianceHubMode.set(!allianceHubMode.get());
          controller1.vibrate(1, 1, 250);
          controller2.vibrate(1, 1, 250);
        },
        true,
        BooleanSurface.BACK);

    controller2.registerOnPressedCallback(robot.gripper::toggle, true, BooleanSurface.DPAD_RIGHT);

    // Mode specific controls
    controller2.registerOnPressedCallback(
        () -> {
          firstTime = false;
          robot.clearFutureEvents();
          TurretBotPosition position = botPositionFor(BooleanSurface.B);
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position, tippedMode.get());
              });
        },
        true,
        BooleanSurface.B);
    controller2.registerOnPressedCallback(
        () -> {
          firstTime = false;
          robot.clearFutureEvents();
          TurretBotPosition position = botPositionFor(BooleanSurface.X);
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position, tippedMode.get());
              });
        },
        true,
        BooleanSurface.X);

    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          TurretBotPosition position = botPositionFor(BooleanSurface.Y);
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position, tippedMode.get());
              });
        },
        true,
        BooleanSurface.Y);
    controller1.registerOnPressedCallback(
        () -> {
          firstTime = false;
          robot.clearFutureEvents();
          robot.intake.stop();
          robot.goToPosition(TurretBotPosition.TEAM_MARKER_GRAB_POSITION, tippedMode.get());
        },
        true,
        BooleanSurface.DPAD_LEFT);
    controller1.registerOnPressedCallback(
        () -> {
          firstTime = false;
          robot.clearFutureEvents();
          TurretBotPosition position = TurretBotPosition.TEAM_MARKER_DEPOSIT_POSITION;
          robot.afterTimedAction(
              robot.grabTeamMarker(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position, tippedMode.get());
              });
        },
        true,
        BooleanSurface.DPAD_UP);
    controller2.registerOnPressedCallback(
        () -> tippedMode.set(!tippedMode.get()), true, BooleanSurface.DPAD_DOWN);
    robot.turret.turnToFront();
    robot.lift.setArmOnePosition(0);
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
    hardwarePipeline.process(initializedHardware, new StateFilterResult(robotObject));
  }

  @Override
  public void onLoop() {
    if (tapeMeasureMode.get()) {
      robot.tapeMeasure.adjustPitch(controller1.leftStickY());
      robot.tapeMeasure.adjustYaw(controller1.leftStickX());
      robot.tapeMeasure.setLengthRate(controller1.leftTrigger() - controller1.rightTrigger());
    } else {
      double turnValue = controller1.rightStickX();
      double speed = halfSpeed.get() ? 0.5 : 0.9;
      robot.drivetrain.driveBySticks(
          controller1.leftStickX() * speed, controller1.leftStickY() * speed, turnValue * speed);
    }
    if (controller2.rightStickY() < -0.02 || controller2.rightStickY() > 0.02) {
      robot.lift.setArmTwoPosition(
          robot.lift.getState().second
              + (-controller2.rightStickY() * MAX_SECOND_JOINT_ADJUSTMENT));
    }

    if (controller2.leftStickX() < -0.02 || controller2.leftStickX() > 0.02) {
      robot.turretAdjustment.getAndAdd(controller2.leftStickX() / 2);
      robot.syncPosition(tippedMode.get());
    }

    if (controller2.leftStickY() < -0.02 || controller2.leftStickY() > 0.02) {
      robot.firstJointAdjustment.getAndAdd(controller2.leftStickY() / 2);
      robot.syncPosition(tippedMode.get());
    }
  }

  @Override
  public void onStop() {
    futures.forEach(future -> future.cancel(false));
    futures.clear();
    robot.clearFutureEvents();
  }

  private TurretBotPosition botPositionFor(BooleanSurface input) {
    switch (input) {
      case A:
        return TurretBotPosition.INTAKE_POSITION;
      case B:
        return allianceHubMode.get()
            ? TurretBotPosition.ALLIANCE_BOTTOM_POSITION
            : tippedMode.get()
                ? TurretBotPosition.TIPPED_CLOSE_POSITION
                : TurretBotPosition.SHARED_CLOSE_POSITION;
      case X:
        return allianceHubMode.get()
            ? TurretBotPosition.ALLIANCE_MIDDLE_POSITION
            : tippedMode.get()
                ? TurretBotPosition.TIPPED_FAR_POSITION
                : TurretBotPosition.SHARED_FAR_POSITION;
      case Y:
        return allianceHubMode.get()
            ? TurretBotPosition.ALLIANCE_TOP_POSITION
            : tippedMode.get()
                ? TurretBotPosition.TIPPED_MIDDLE_POSITION
                : TurretBotPosition.SHARED_MIDDLE_POSITION;
      default:
        return null;
    }
  }

  private Runnable carouselMoveForAlliance(Alliance alliance) {
    switch (alliance) {
      case RED:
        return robot.carouselSpinner::spinForward;
      case BLUE:
        return robot.carouselSpinner::spinBackward;
      default:
        return null;
    }
  }
}
