package org.firstinspires.ftc.teamcode.hardware.mechanisms.lifts;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface ISingleJointAngularLift extends Component, StatefulMechanism<Integer> {
    void goToPosition(int ticks);
}
