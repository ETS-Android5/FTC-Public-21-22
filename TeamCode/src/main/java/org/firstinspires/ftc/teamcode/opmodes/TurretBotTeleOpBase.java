package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class TurretBotTeleOpBase extends EnhancedTeleOp {
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.0055; // 1.5 degrees

    /*
    * The shared alliance hub basically has 12 places to put blocks before they start stacking up
    * Those places are organized as follows
    *                    Field wall
    *                       1|1
    *                      2 | 2
    *                     3  |  3
    *                    4 12|12 4
    *Blue alliance side 5  11|11  5  Red alliance side
    *                    6 10|10 6
    *                     7  |  7
    *                      8 | 8
    *                       9|9
    * */

    private static final double[][] SHARED_POSITION_LIST = {
            {
                20, 0.43 // pos 5
            },
            {
                0, 0.47 // pos 4
            },
            {
                -30, 0.33 // pos 6
            },
            {
                -230, 0.15 // pos 7
            },
            {
                0, 0.57 // pos 3
            },
            {
                -230, 0.15 // pos 8
            },
            {
                0, 0.59 // pos 2
            },
            {
                -230, 0.15 // pos 9
            },
            {
                0, 0.65 // pos 1
            },
            {
                -230, 0.15 // pos 10
            },
            {
                0, 0.5 // pos 12
            },
            {
                -70, 0.37 // pos 11
            },
    };

  private final TurretBot robot;

  private final AtomicBoolean halfSpeed = new AtomicBoolean(false);

  private final AtomicBoolean allianceHubMode = new AtomicBoolean(false);
  private final AtomicBoolean tippedMode = new AtomicBoolean(false);

  private final AtomicInteger sharedDropProgress = new AtomicInteger(0);

  public TurretBotTeleOpBase(Alliance alliance) {
    super(new TurretBot(alliance));
    this.robot = (TurretBot) super.robotObject;
  }

  private static double THIRD_MANIPULATION(double in) {
    return Math.pow(in, 3);
  }

  @Override
  public void onInitPressed() {}

  @Override
  public void initLoop() {}

  @Override
  public void onStartPressed() {
    robot
        .getExecutorService()
        .schedule(
            () -> {
              controller1.vibrate(0.25, 0.25, 5000);
              controller2.vibrate(0.25, 0.25, 5000);
            },
            75,
            TimeUnit.SECONDS);

    robot
        .getExecutorService()
        .schedule(
            () -> {
              controller1.vibrate(0.5, 0.5, 5000);
              controller2.vibrate(0.5, 0.5, 5000);
            },
            80,
            TimeUnit.SECONDS);

    robot
        .getExecutorService()
        .schedule(
            () -> {
              controller1.vibrate(1, 1, 5000);
              controller2.vibrate(1, 1, 5000);
            },
            85,
            TimeUnit.SECONDS);

    controller1.setManipulation(
        TurretBotTeleOpBase::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);

    controller1.registerOnPressedCallback(
        () -> halfSpeed.set(!halfSpeed.get()), true, BooleanSurface.X);

    controller1.registerOnPressedCallback(this::onAPressed, true, BooleanSurface.A);

    controller2.registerOnPressedCallback(this::onAPressed, true, BooleanSurface.A);

    controller2.registerOnPressedCallback(
        robot.intake::toggleIntaking, true, BooleanSurface.RIGHT_BUMPER);
    controller2.registerOnPressedCallback(
        robot.intake::toggleOuttaking, true, BooleanSurface.LEFT_BUMPER);

    controller2.registerOnPressedCallback(
        carouselMoveForAlliance(robot.getAlliance()), true, BooleanSurface.LEFT_STICK);

    controller2.registerOnPressedCallback(
        () -> robot.setAlliance(robot.getAlliance().opposite()), true, BooleanSurface.RIGHT_STICK);

    controller2.registerOnPressedCallback(
        () -> allianceHubMode.set(!allianceHubMode.get()), true, BooleanSurface.BACK);

    controller2.registerOnPressedCallback(robot.gripper::toggle, true, BooleanSurface.DPAD_RIGHT);

    // Mode specific controls
    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          TurretBotPosition position = botPositionFor(BooleanSurface.B);
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position);
              });
        },
        true,
        BooleanSurface.B);
    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          TurretBotPosition position = botPositionFor(BooleanSurface.X);
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position);
              });
        },
        true,
        BooleanSurface.X);

    controller2.registerOnPressedCallback(this::onController2YPressed, true, BooleanSurface.Y);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.clearFutureEvents();
            robot.intake.stop();
            robot.goToPosition(TurretBotPosition.TEAM_MARKER_GRAB_POSITION);
          } else if (sharedDropProgress.get() < SHARED_POSITION_LIST.length) {
              robot.clearFutureEvents();
              robot.afterTimedAction(robot.grabFreight(), () -> {
                  robot.intake.stop();
                  robot.goToSharedPosition(
                          SHARED_POSITION_LIST[sharedDropProgress.get()],
                          sharedDropProgress::incrementAndGet);
              });
          } else {
              onController2YPressed();
          }
        },
        true,
        BooleanSurface.DPAD_LEFT);
    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          TurretBotPosition position = botPositionFor(BooleanSurface.DPAD_UP);
          robot.afterTimedAction(
              position == TurretBotPosition.TEAM_MARKER_DEPOSIT_POSITION
                  ? robot.grabTeamMarker()
                  : robot.grabFreight(),
              () -> {
                robot.intake.stop();
                robot.goToPosition(position);
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
    double turnValue = controller1.rightStickX();
    double speed = halfSpeed.get() ? 0.5 : 1;
    robot.drivetrain.driveBySticks(
        controller1.leftStickX() * speed, controller1.leftStickY() * speed, turnValue * speed);
    if (controller2.rightStickY() < -0.02 || controller2.rightStickY() > 0.02) {
        robot.onTrim();
      robot.lift.setArmTwoPosition(
          robot.lift.getState().second
              + (-controller2.rightStickY() * MAX_SECOND_JOINT_ADJUSTMENT));
    }
  }

  @Override
  public void onStop() {}

  private void onAPressed() {
    robot.clearFutureEvents();
    robot.afterTimedAction(
        robot.dropFreight() + (allianceHubMode.get() ? 750 : 0),
        () -> robot.goToPosition(TurretBotPosition.INTAKE_POSITION));
  }

  private void onController2YPressed() {
      robot.clearFutureEvents();
      TurretBotPosition position = botPositionFor(BooleanSurface.Y);
      robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                  robot.intake.stop();
                  robot.goToPosition(position);
              });
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
      case DPAD_UP:
        return allianceHubMode.get()
            ? TurretBotPosition.TEAM_MARKER_DEPOSIT_POSITION
            : TurretBotPosition.INTAKE_HOVER_POSITION;
      default:
        return null;
    }
  }

  private Runnable carouselMoveForAlliance(Alliance alliance) {
    switch (alliance) {
      case RED:
        return robot.carouselSpinner::spinBackward;
      case BLUE:
        return robot.carouselSpinner::spinForward;
      default:
        return null;
    }
  }
}
