package org.firstinspires.ftc.teamcode.core.sounds;

import android.media.MediaPlayer;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.concurrent.atomic.AtomicReference;

public class TrackManager {
    private final AtomicReference<MediaPlayer> mediaPlayer = new AtomicReference<>();

    public void playTrack(int sound, boolean loop, long loopForMs) {
        MediaPlayer player = getMediaPlayer(sound);
        player.setLooping(loop);
        player.setOnCompletionListener((player1) -> {
            player1.release();
            mediaPlayer.compareAndSet(player1, null);
        });
        MediaPlayer prevPlayer = mediaPlayer.getAndSet(player);
        if (prevPlayer != null) {
            prevPlayer.stop();
            prevPlayer.release();
        }
        player.start();
        if (loopForMs != 0) {
            new Thread(() -> {
                try {
                  Thread.sleep(loopForMs);
                } catch (InterruptedException ignored) {
                }
                MediaPlayer killablePlayer = mediaPlayer.getAndSet(null);
                if (killablePlayer != null) {
                  killablePlayer.stop();
                  killablePlayer.release();
                }
            }).start();
        }
    }

    public void playSound(int sound) {
        MediaPlayer player = getMediaPlayer(sound);
        player.setOnCompletionListener(MediaPlayer::release);
        player.start();
    }

    private static MediaPlayer getMediaPlayer(int sound) {
        MediaPlayer player = MediaPlayer.create(FtcRobotControllerActivity.getInstance().getApplicationContext(), sound);
        player.setVolume(1, 1);
        return player;
    }
}
