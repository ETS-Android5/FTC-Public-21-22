package org.firstinspires.ftc.teamcode.opmodes.auto.red.warehouse;

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
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBot;
import org.firstinspires.ftc.teamcode.hardware.robots.turretbot.TurretBotPosition;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "CB_AUTO_RedWBarrier")
@SuppressWarnings("unused")
public class AutoRedWarehouseEndBarrier extends EnhancedAutonomous {
    private final TurretBot robot;

    private static final double FRONT_DISTANCE_FROM_WALL_0 = 177.8;
    private static final double FRONT_DISTANCE_FROM_WALL_1 = 736.6;
    private static final double FRONT_DISTANCE_FROM_WALL_BOTTOM = 750;
    private static final double FRONT_DISTANCE_FROM_WALL_MIDDLE = 750;
    private static final double FRONT_DISTANCE_FROM_WALL_TOP = 850;
    private static final double RIGHT_DISTANCE_FROM_WALL_0 = 460;

    public AutoRedWarehouseEndBarrier() {
        super(new TurretBot(Alliance.RED, TurretBot.AUTO_FIRST_JOINT_OFFSET, true));
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
        robot.turnToHeading(35, 0.5, super::opModeIsActive, super::processChanges);
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
        robot.setAlliance(Alliance.BLUE);
        robot.goToPosition(TurretBotPosition.ALLIANCE_BOTTOM_POSITION);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            processChanges();
        }
        robot.setAlliance(Alliance.RED);
        double preMoveFrontDistance = InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName());
        robot.runAtHeadingUntilCondition(
                35,
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
        retreat(preMoveFrontDistance);
    }

    private void handleMiddlePosition() {
        robot.setAlliance(Alliance.BLUE);
        robot.goToPosition(TurretBotPosition.ALLIANCE_MIDDLE_POSITION);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            processChanges();
        }
        robot.setAlliance(Alliance.RED);
        double preMoveFrontDistance = InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName());
        robot.runAtHeadingUntilCondition(
                35,
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
        retreat(preMoveFrontDistance);
    }

    private void handleTopPosition() {
        robot.setAlliance(Alliance.BLUE);
        robot.goToPosition(TurretBotPosition.ALLIANCE_TOP_POSITION);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            processChanges();
        }
        robot.setAlliance(Alliance.RED);
        double preMoveFrontDistance = InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName());
        robot.runAtHeadingUntilCondition(
                35,
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
        retreat(preMoveFrontDistance);
    }

    private void retreat(double preMoveFrontDistance) {
        robot.runAtHeadingUntilCondition(
                35,
                90,
                Direction.FORWARD,
                () -> InterpolatablePipe.getInstance().currentDataPointOf(robot.frontRange.getName()) <= preMoveFrontDistance,
                () -> 0.25,
                super::opModeIsActive,
                super::processChanges
        );
        robot.grabTeamMarker();
        robot.goToPosition(TurretBotPosition.INTAKE_POSITION);
        robot.turnToHeading(90, 0.5, super::opModeIsActive, super::processChanges);
        robot.drivetrain.driveBySticks(-0.5, 0, 0);
        robot.frontRange.stopSampling();
        robot.rightRange.startSampling();
        processChanges();
        while (opModeIsActive()
                && InterpolatablePipe.getInstance().currentDataPointOf(robot.rightRange.getName()) <= RIGHT_DISTANCE_FROM_WALL_0) {
            processChanges();
        }
        robot.drivetrain.setAllPower(-1);
        robot.rightRange.stopSampling();
        processChanges();
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 1250) {
            processChanges();
        }
        robot.drivetrain.setAllPower(0);
        while (opModeIsActive()) {
            processChanges();
        }
    }

    @Override
    public void onStop() {}
}
