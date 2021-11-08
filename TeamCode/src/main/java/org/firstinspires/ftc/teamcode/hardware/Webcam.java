package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.ImageFormat;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;

public class Webcam {
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureRequest request;

    public Mat grabFrame(HardwareMap hardwareMap) {
        if (request != null) {
            if (cameraManager == null) {
                cameraManager = ClassFactory.getInstance().getCameraManager();
            }
            if (cameraName == null) {
                cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            }
            if (camera == null) {
                camera = cameraManager.requestPermissionAndOpenCamera(new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS), cameraName, null);
            }
            CameraCharacteristics characteristics = cameraName.getCameraCharacteristics();
            Size size = characteristics.getDefaultSize(ImageFormat.YUY2);
            int fps = characteristics.getMaxFramesPerSecond(ImageFormat.YUY2, size);
            try {
                request = camera.createCaptureRequest(ImageFormat.YUY2, size, fps);
            } catch (CameraException e) {
                return null;
            }
        }
        return null;
    }


}
