package org.firstinspires.ftc.teamcode.hardware.detection.vision;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;

public class Webcam implements FtcCamera {
  private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
  private static final String CAMERA_NAME = "Webcam 1";
  private final Handler callbackHandler = CallbackLooper.getDefault().getHandler();

  @Hardware(name = CAMERA_NAME)
  public WebcamName cameraName;

  private CameraManager cameraManager;
  private Camera camera;
  private Bitmap bitmap;
  private CameraCaptureSession session;

  @Override
  public void init() {
    if (cameraName == null) return;
    if (cameraManager == null) {
      cameraManager = ClassFactory.getInstance().getCameraManager();
    }
    if (camera == null) {
      camera =
          cameraManager.requestPermissionAndOpenCamera(
              new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS), cameraName, null);
    }
    CameraCharacteristics characteristics = cameraName.getCameraCharacteristics();
    Size size = characteristics.getDefaultSize(ImageFormat.YUY2);
    int fps = characteristics.getMaxFramesPerSecond(ImageFormat.YUY2, size);
    final ContinuationSynchronizer<CameraCaptureSession> synchronizer =
        new ContinuationSynchronizer<>();
    try {
      camera.createCaptureSession(
          Continuation.create(
              callbackHandler,
              new CameraCaptureSession.StateCallbackDefault() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                  try {
                    final CameraCaptureRequest captureRequest =
                        camera.createCaptureRequest(ImageFormat.YUY2, size, fps);
                    session.startCapture(
                        captureRequest,
                        (_u, _uu, cameraFrame) -> {
                          Bitmap bmp = captureRequest.createEmptyBitmap();
                          cameraFrame.copyToBitmap(bmp);
                          bitmap = bmp;
                        },
                        Continuation.create(callbackHandler, (_u, _uu, _uuu) -> {}));
                    synchronizer.finish(session);
                  } catch (CameraException | RuntimeException e) {
                    e.printStackTrace();
                    session.close();
                    synchronizer.finish(null);
                  }
                }
              }));
    } catch (CameraException e) {
      e.printStackTrace();
      synchronizer.finish(null);
    }
    try {
      synchronizer.await();
    } catch (InterruptedException e) {
      e.printStackTrace();
      Thread.currentThread().interrupt();
    }
    session = synchronizer.getValue();
    while (bitmap == null) {}
  }

  @Override
  public void deinit() {
    if (session != null) {
      session.stopCapture();
      session.close();
      session = null;
    }
    if (camera != null) {
      camera.close();
      camera = null;
    }
  }

  @Override
  public Mat grabFrame() {
    if (cameraName == null) return null;

    Mat mat = new Mat();
    Bitmap bmp32 =
        Bitmap.createScaledBitmap(
                bitmap,
                (int) Math.round(bitmap.getWidth() / 2.5),
                (int) Math.round(bitmap.getHeight() / 2.5),
                false)
            .copy(Bitmap.Config.RGB_565, true);
    Utils.bitmapToMat(bmp32, mat);
    return mat;
  }
}
