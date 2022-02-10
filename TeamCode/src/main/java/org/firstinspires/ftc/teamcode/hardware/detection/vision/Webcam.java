package org.firstinspires.ftc.teamcode.hardware.detection.vision;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.util.Log;

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
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.core.annotations.PostInit;
import org.firstinspires.ftc.teamcode.core.annotations.hardware.Hardware;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

public class Webcam implements FtcCamera {
  private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
  private static final String CAMERA_NAME = "CB_AUTO_Webcam 1";
  private final Handler callbackHandler = CallbackLooper.getDefault().getHandler();
  private final BlockingQueue<Bitmap> frameQueue = new LinkedBlockingQueue<>(1);
  private final BlockingQueue<Boolean> initializationQueue = new LinkedBlockingQueue<>(1);

  @Hardware(name = CAMERA_NAME)
  public WebcamName cameraName;

  private CameraManager cameraManager;
  private Camera camera;
  private CameraCaptureSession session;
  private CameraCharacteristics characteristics;

  @PostInit(argType = WebcamName.class)
  @SuppressWarnings("unused")
  public void init(WebcamName arg) {
    init();
    Log.d("WEBCAM", "INITIALIZED");
  }

  @Override
  public void init() {
    try {
      if (cameraName == null) return;
      if (cameraManager == null) {
        synchronized (this) {
          cameraManager = ClassFactory.getInstance().getCameraManager();
        }
      }
      if (camera == null) {
        synchronized (this) {
          camera =
              cameraManager.requestPermissionAndOpenCamera(
                  new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS), cameraName, null);
        }
      }
      synchronized (this) {
        characteristics = cameraName.getCameraCharacteristics();
      }
      initializationQueue.add(true);
    } catch (Exception e) {
      initializationQueue.add(false);
      Log.e("WEBCAM", "ERROR", e);
    }
  }

  @Override
  public void start() {
    boolean successfullyInitialized = false;
    try {
      successfullyInitialized = initializationQueue.take();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    if (successfullyInitialized) {
      try {
        ContinuationSynchronizer<CameraCaptureSession> synchronizer =
            new ContinuationSynchronizer<>();
        Size size = characteristics.getDefaultSize(ImageFormat.YUY2);
        int fps = characteristics.getMaxFramesPerSecond(ImageFormat.YUY2, size);
        synchronized (this) {
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
                                try {
                                  frameQueue.put(bmp);
                                  deinit();
                                } catch (InterruptedException e) {
                                  Log.e("WEBCAM", "ERROR", e);
                                }
                              },
                              Continuation.create(callbackHandler, (_u, _uu, _uuu) -> {}));
                          synchronizer.finish(session);
                        } catch (CameraException | RuntimeException e) {
                          Log.e("WEBCAM", "ERROR", e);
                          session.close();
                          synchronizer.finish(null);
                        }
                      }
                    }));
          } catch (CameraException e) {
            Log.e("WEBCAM", "ERROR", e);
            synchronizer.finish(null);
          }
        }
        try {
          synchronizer.await();
        } catch (InterruptedException e) {
          Log.e("WEBCAM", "ERROR", e);
          Thread.currentThread().interrupt();
        }
        synchronized (this) {
          session = synchronizer.getValue();
        }
      } catch (Exception e) {
        Log.e("WEBCAM", "ERROR", e);
      }
    }
  }

  @Override
  public void deinit() {
    try {
      if (session != null) {
        session.stopCapture();
        session.close();
        synchronized (this) {
          session = null;
        }
      }
      if (camera != null) {
        camera.close();
        synchronized (this) {
          camera = null;
        }
      }
    } catch (Exception e) {
      Log.e("WEBCAM", "ERROR", e);
    }
  }

  @Override
  public Mat grabFrame() {
    if (cameraName == null) return null;
    Mat mat = new Mat();
    try {
      Bitmap bmp = frameQueue.take();
      saveBitmap(bmp);
      Utils.bitmapToMat(bmp, mat);
    } catch (InterruptedException e) {
      return null;
    }
    return mat;
  }

  private void saveBitmap(Bitmap bitmap) {
    File file = new File(AppUtil.ROBOT_DATA_DIR, "Capture.jpg");
    try {
      try (FileOutputStream outputStream = new FileOutputStream(file)) {
        bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
      }
    } catch (IOException ignored) {
    }
  }
}
