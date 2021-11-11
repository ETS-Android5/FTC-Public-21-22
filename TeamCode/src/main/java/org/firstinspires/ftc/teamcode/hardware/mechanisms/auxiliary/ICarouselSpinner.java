package org.firstinspires.ftc.teamcode.hardware.mechanisms.auxiliary;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.StatefulMechanism;

public interface ICarouselSpinner extends Component, StatefulMechanism<CarouselSpinnerState> {
    void spinForward();
    void spinBackward();
}
