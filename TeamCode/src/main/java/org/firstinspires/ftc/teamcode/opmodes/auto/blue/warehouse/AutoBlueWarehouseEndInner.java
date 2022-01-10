package org.firstinspires.ftc.teamcode.opmodes.auto.blue.warehouse;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.annotations.hardware.Direction;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.RunMode;
import org.firstinspires.ftc.teamcode.core.game.related.Alliance;
import org.firstinspires.ftc.teamcode.core.hardware.pipeline.InterpolatablePipe;
import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary.Turret;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "CB_AUTO_BlueWInner")
@SuppressWarnings("unused")
public class AutoBlueWarehouseEndInner extends EnhancedAutonomous {
    private final TurretBot robot;

    private static final double FRONT_DISTANCE_FROM_WALL_0 = 177.8;
    private static final double FRONT_DISTANCE_FROM_WALL_1 = 736.6;
    private static final double FRONT_DISTANCE_FROM_WALL_BOTTOM = 650;
    private static final double FRONT_DISTANCE_FROM_WALL_MIDDLE = 650;
    private static final double FRONT_DISTANCE_FROM_WALL_TOP = 850;
    private static final double LEFT_DISTANCE_FROM_WALL_0 = 50.8;
    private static final double LEFT_DISTANCE_FROM_WALL_1 = 609.6;

    public AutoBlueWarehouseEndInner() {
        super(new TurretBot(Alliance.BLUE, TurretBot.AUTO_FIRST_JOINT_OFFSET, true));
        this.robot = (TurretBot) super.robotObject;
    }

    @Override
    public void onInitPressed() {
        OpenCVWrapper.load();
    }

    @Override
    public void onStartPressed() {
        AtomicReference<Mat> img = new AtomicReference<>();
        AtomicReference<TeamMarkerPosition> teamMarkerPosition =
                new AtomicReference<>(TeamMarkerPosition.RIGHT);
        Thread worker =
                new Thread(
                        () -> {
                            robot.webcam.start();
                            img.set(robot.webcam.grabFrame());
                            new Thread(robot.webcam::deinit).start();
                        });
        worker.start();
        robot.frontRange.startSampling();
        robot.gyro.startSampling();
        robot.drivetrain.setRunMode(RunMode.RUN_USING_ENCODER);
        robot.lift.setArmTwoPosition(0.8);
        int delay = robot.grabFreight();
        processChanges();
        sleep(delay);
        robot.lift.setArmOnePosition(robot.firstJointOffset + 550);
        processChanges();
        try {
            worker.join();
            worker =
                    new Thread(
                            () -> teamMarkerPosition.set(
                                    new TeamMarkerPositionDetector()
                                            .calculateTeamMarkerPosition(
                                                    img.get(), CameraPosition.REAR_LOW_AND_CENTERED)));
            worker.start();
        } catch (InterruptedException e) {
            Log.e("TURRETBOT", "ERROR", e);
        }
        robot.runAtHeadingUntilCondition(
                0,
                90,
                Direction.REVERSE,
                () -> InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) >= FRONT_DISTANCE_FROM_WALL_0,
                () -> 0.25,
                super::opModeIsActive,
                super::processChanges
        );
        robot.turnToHeading(-35, 0.5, super::opModeIsActive, super::processChanges);
        try {
            worker.join();
        } catch (InterruptedException e) {
            Log.e("TURRETBOT", "ERROR", e);
        }
        switch (teamMarkerPosition.get()) {
            case LEFT:
                Log.d("TURRETBOT", "SAW LEFT");
                handleBottomPosition();
                break;
            case CENTER:
                Log.d("TURRETBOT", "SAW CENTER");
                handleMiddlePosition();
                break;
            case RIGHT:
                Log.d("TURRETBOT", "SAW RIGHT");
                handleTopPosition();
                break;
        }
    }

    private void handleBottomPosition() {
        robot.setAlliance(Alliance.RED);
        robot.goToPosition(TurretBotPosition.ALLIANCE_BOTTOM_POSITION);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            processChanges();
        }
        robot.setAlliance(Alliance.BLUE);
        double preMoveFrontDistance = InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName());
        robot.runAtHeadingUntilCondition(
                -35,
                90,
                Direction.REVERSE,
                () -> InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) >= FRONT_DISTANCE_FROM_WALL_BOTTOM,
                () -> 0.25,
                super::opModeIsActive,
                super::processChanges
        );
        int delay = robot.dropFreight();
        processChanges();
        sleep(delay);
        retreat(preMoveFrontDistance + 320);
    }

    private void handleMiddlePosition() {
        robot.setAlliance(Alliance.RED);
        robot.goToPosition(TurretBotPosition.ALLIANCE_MIDDLE_POSITION);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            processChanges();
        }
        robot.setAlliance(Alliance.BLUE);
        double preMoveFrontDistance = InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName());
        robot.runAtHeadingUntilCondition(
                -35,
                90,
                Direction.REVERSE,
                () -> InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) >= FRONT_DISTANCE_FROM_WALL_MIDDLE,
                () -> 0.25,
                super::opModeIsActive,
                super::processChanges
        );
        int delay = robot.dropFreight();
        processChanges();
        sleep(delay);
        retreat(preMoveFrontDistance + 360);
    }

    private void handleTopPosition() {
        robot.setAlliance(Alliance.RED);
        robot.goToPosition(TurretBotPosition.ALLIANCE_TOP_POSITION);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            processChanges();
        }
        robot.setAlliance(Alliance.BLUE);
        double preMoveFrontDistance = InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName());
        robot.runAtHeadingUntilCondition(
                -35,
                90,
                Direction.REVERSE,
                () -> InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) >= FRONT_DISTANCE_FROM_WALL_TOP,
                () -> 0.25,
                super::opModeIsActive,
                super::processChanges
        );
        int delay = robot.dropFreight();
        processChanges();
        sleep(delay);
        retreat(preMoveFrontDistance + 180);
    }

    private void retreat(double preMoveFrontDistance) {
        robot.turret.turnToPosition(Turret.TICKS_LEFT);
        long turretStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - turretStart < 2000) {
            processChanges();
        }
        robot.runAtHeadingUntilCondition(
                -35,
                90,
                Direction.FORWARD,
                () -> InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) <= preMoveFrontDistance,
                () -> 0.25,
                super::opModeIsActive,
                super::processChanges
        );
        robot.grabTeamMarker();
        robot.goToPosition(TurretBotPosition.INTAKE_POSITION);
        robot.turnToHeading(-90, 0.5, super::opModeIsActive, super::processChanges);
        long intakeStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - intakeStart < 1000) {
            processChanges();
        }
        robot.drivetrain.driveBySticks(-0.5, 0, 0);
        robot.frontRange.stopSampling();
        robot.leftRange.startSampling();
        processChanges();
        while (opModeIsActive()
                && InterpolatablePipe.getInstance().currentDataPointOf(robot.leftRange.getName()) >= LEFT_DISTANCE_FROM_WALL_0) {
            processChanges();
        }
        robot.drivetrain.setAllPower(-0.25);
        robot.frontRange.startSampling();
        robot.leftRange.stopSampling();
        processChanges();
        while (opModeIsActive()
                && InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) >= FRONT_DISTANCE_FROM_WALL_1) {
            processChanges();
        }
        robot.drivetrain.driveBySticks(1, 0, 0);
        robot.frontRange.stopSampling();
        robot.leftRange.startSampling();
        processChanges();
        while (opModeIsActive()
                && InterpolatablePipe.getInstance().currentDataPointOf(robot.leftRange.getName()) <= LEFT_DISTANCE_FROM_WALL_1) {
            processChanges();
        }
        robot.drivetrain.setAllPower(0);
        processChanges();
    }

    @Override
    public void onStop() {}
}
