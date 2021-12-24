package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.controller.BooleanSurface;
import org.firstinspires.ftc.teamcode.core.controller.ScalarSurface;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.MotorTrackerPipe;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.StateFilterResult;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedTeleOp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassisPosition;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "Red")
public class RedTeleOp extends EnhancedTeleOp {
  private static final double MAX_TURRET_ADJUSTMENT = 1; // 1 tick or 1.25 degrees
  private static final double MAX_FIRST_JOINT_ADJUSTMENT = 2.5; // ticks
  private static final double MAX_SECOND_JOINT_ADJUSTMENT = 0.0055; // 1.5 degrees

  private static final int firstJointOffset = 230;

  private final NewChassis robot;

  private final AtomicBoolean halfSpeed = new AtomicBoolean(false);

  private final AtomicBoolean allianceHubMode = new AtomicBoolean(false);
  private final AtomicBoolean tippedMode = new AtomicBoolean(false);

  private boolean previouslyTrimming = false;

  public RedTeleOp() {
    super(new NewChassis(Alliance.RED));
    this.robot = (NewChassis) super.robotObject;
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
    controller1.setManipulation(RedTeleOp::THIRD_MANIPULATION, ScalarSurface.LEFT_STICK_Y);

    controller1.registerOnPressedCallback(
        () -> halfSpeed.set(!halfSpeed.get()), true, BooleanSurface.X);

      controller1.registerOnPressedCallback(
              () -> {
                  robot.clearFutureEvents();
                  robot.afterTimedAction(
                          robot.dropFreight(), () -> robot.goToPosition(NewChassisPosition.INTAKE_POSITION));
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
        () -> {
          boolean allianceHubModeTmp = allianceHubMode.get();
          allianceHubMode.set(!allianceHubModeTmp);
        },
        true,
        BooleanSurface.BACK);

    controller2.registerOnPressedCallback(robot.gripper::toggle, true, BooleanSurface.DPAD_RIGHT);

    // Mode specific controls
    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          boolean inAllianceHubMode = allianceHubMode.get();
          boolean inTippedMode = tippedMode.get();
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.outtakeIfNecessary();
                if (inAllianceHubMode) {
                  robot.goToPosition(NewChassisPosition.ALLIANCE_BOTTOM_POSITION);
                } else {
                  if (inTippedMode) {
                    robot.goToPosition(NewChassisPosition.TIPPED_CLOSE_POSITION);
                  } else {
                    robot.goToPosition(NewChassisPosition.SHARED_CLOSE_POSITION);
                  }
                }
              });
        },
        true,
        BooleanSurface.B);
    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          boolean inAllianceHubMode = allianceHubMode.get();
          boolean inTippedMode = tippedMode.get();
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.outtakeIfNecessary();
                if (inAllianceHubMode) {
                  robot.goToPosition(NewChassisPosition.ALLIANCE_MIDDLE_POSITION);
                } else {
                  if (inTippedMode) {
                    robot.goToPosition(NewChassisPosition.TIPPED_MIDDLE_POSITION);
                  } else {
                    robot.goToPosition(NewChassisPosition.SHARED_MIDDLE_POSITION);
                  }
                }
              });
        },
        true,
        BooleanSurface.X);

    controller2.registerOnPressedCallback(
        () -> {
          robot.clearFutureEvents();
          boolean inAllianceHubMode = allianceHubMode.get();
          boolean inTippedMode = tippedMode.get();
          robot.afterTimedAction(
              robot.grabFreight(),
              () -> {
                robot.outtakeIfNecessary();
                if (inAllianceHubMode) {
                  robot.goToPosition(NewChassisPosition.ALLIANCE_TOP_POSITION);
                } else {
                  if (inTippedMode) {
                    robot.goToPosition(NewChassisPosition.TIPPED_FAR_POSITION);
                  } else {
                    robot.goToPosition(NewChassisPosition.SHARED_FAR_POSITION);
                  }
                }
              });
        },
        true,
        BooleanSurface.Y);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.clearFutureEvents();
            robot.outtakeIfNecessary();
            robot.goToPosition(NewChassisPosition.TEAM_MARKER_GRAB_POSITION);
          }
        },
        true,
        BooleanSurface.DPAD_LEFT);
    controller2.registerOnPressedCallback(
        () -> {
          if (allianceHubMode.get()) {
            robot.clearFutureEvents();
            robot.outtakeIfNecessary();
            robot.afterTimedAction(
                robot.grabTeamMarker(),
                () -> robot.goToPosition(NewChassisPosition.TEAM_MARKER_DEPOSIT_POSITION));
          }
        },
        true,
        BooleanSurface.DPAD_UP);
    controller2.registerOnPressedCallback(
        () -> tippedMode.set(!tippedMode.get()), true, BooleanSurface.DPAD_DOWN);
    robot.turret.turnToFront();
    robot.lift.setArmOnePosition(firstJointOffset);
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
}
