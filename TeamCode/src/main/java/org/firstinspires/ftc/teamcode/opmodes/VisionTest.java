package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.opmodes.EnhancedAutonomous;
import org.firstinspires.ftc.teamcode.cv.ITeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.hardware.robots.NewChassis;
import org.opencv.core.Mat;

@Autonomous(name="CB_AUTO_VisionTest")
public class VisionTest extends EnhancedAutonomous {

    private final NewChassis robot;

    public VisionTest() {
        super(new NewChassis());
        this.robot = (NewChassis) robotObject;
    }

    @Override
    public void onInitPressed() {
        OpenCVWrapper.load();
    }

    @Override
    public void onStartPressed() {
        long start = System.nanoTime();
        robot.webcam.init();
        processChanges();
        ITeamMarkerPositionDetector markerPositionDetector = new TeamMarkerPositionDetector();
        Mat frame = robot.webcam.grabFrame();
        robot.webcam.deinit();
        TeamMarkerPosition teamMarkerPosition = markerPositionDetector.calculateTeamMarkerPosition(frame);
        long end = System.nanoTime();
        Log.d("TIME", "" + (start - end));
        Log.d("MARKER POSITION", teamMarkerPosition.name());
    }

    @Override
    public void onStop() {
        robot.webcam.deinit();
    }
}
