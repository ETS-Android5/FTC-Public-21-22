package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.robots.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.TurretBotPosition;

import java.util.concurrent.atomic.AtomicBoolean;

public class TurretBotTeleOpBase extends EnhancedTeleOp {
  private static final double MAX_TURRET_ADJUSTMENT = 1; // 1 tick or 1.25 degrees
  private static final double MAX_FIRST_JOINT_ADJUSTMENT = 2.5; // ticks
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.0055; // 1.5 degrees

  private final TurretBot robot;

  private final AtomicBoolean halfSpeed = new AtomicBoolean(false);

  private final AtomicBoolean allianceHubMode = new AtomicBoolean(false);
  private final AtomicBoolean tippedMode = new AtomicBoolean(false);

  private boolean previouslyTrimming = false;

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
    controller1.setManipulation(
        TurretBotTeleOpBase::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);

    controller1.registerOnPressedCallback(
        () -> halfSpeed.set(!halfSpeed.get()), true, BooleanSurface.X);

    controller1.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          robot.afterTimedAction(
              robot.dropFreight(), () -> robot.goToPosition(TurretBotPosition.INTAKE_POSITION));
        },
        true,
        BooleanSurface.A);

    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          robot.afterTimedAction(
              robot.dropFreight(), () -> robot.goToPosition(TurretBotPosition.INTAKE_POSITION));
        },
        true,
        BooleanSurface.A);

    controller2.registerOnPressedCallback(
        robot.intake::toggleIntaking, true, BooleanSurface.RIGHT_BUMPER);
    controller2.registerOnPressedCallback(
        robot.intake::toggleOuttaking, true, BooleanSurface.LEFT_BUMPER);

    controller2.registerOnPressedCallback(
        robot.carouselSpinner::spinBackward, true, BooleanSurface.LEFT_STICK);
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
                // robot.outtakeIfNecessary();
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
                // robot.outtakeIfNecessary();
                robot.intake.stop();
                robot.goToPosition(position);
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
                // robot.outtakeIfNecessary();
                robot.intake.stop();
                robot.goToPosition(position);
              });
        },
        true,
        BooleanSurface.Y);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.clearFutureEvents();
            // robot.outtakeIfNecessary();
            robot.intake.stop();
            robot.goToPosition(TurretBotPosition.TEAM_MARKER_GRAB_POSITION);
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
    if (controller2.leftTrigger() > 0.02 || controller2.rightTrigger() > 0.02) {

      robot.turret.turnToDegrees(
          (int)
              Math.round(
                  robot.turret.getState()
                      - (controller2.leftTrigger() * MAX_TURRET_ADJUSTMENT * 2)
                      + (controller2.rightTrigger() * MAX_TURRET_ADJUSTMENT)));
      previouslyTrimming = true;
    } else {
      if (previouslyTrimming) {
        robot.turret.turnToDegrees(
            MotorTrackerPipe.getInstance().getPositionOf(Turret.TURRET_MOTOR_NAME));
        previouslyTrimming = false;
      }
    }
    if (controller2.leftStickY() < -0.02 || controller2.leftStickY() > 0.02) {
      robot.lift.setArmOnePosition(
          (int)
              Math.round(
                  robot.lift.getState().first
                      + (controller2.leftStickY() * MAX_FIRST_JOINT_ADJUSTMENT)));
    }
    if (controller2.rightStickY() < -0.02 || controller2.rightStickY() > 0.02) {
      robot.lift.setArmTwoPosition(
          robot.lift.getState().second
              + (-controller2.rightStickY() * MAX_SECOND_JOINT_ADJUSTMENT));
    }
  }

  @Override
  public void onStop() {}

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
}
