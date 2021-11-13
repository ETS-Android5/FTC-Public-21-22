package org.firstinspires.ftc.teamcode.hardware.robots;

import org.firstinspires.ftc.teamcode.core.hardware.state.Component;
import org.firstinspires.ftc.teamcode.core.hardware.state.State;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.GoBildaMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.drivetrains.IMecanumDrivetrain;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

public class NewChassis implements Component {
    public final IMecanumDrivetrain drivetrain = new GoBildaMecanumDrivetrain();

    @Override
    public String getName() {
        return this.getClass().getName();
    }

    @Override
    public List<? super State> getNextState() {
        List<? super State> states = new LinkedList<>();
        Collections.singletonList(drivetrain)
                .forEach((component) -> component.getNextState().forEach((s) -> states.add((State) s)));
        return states;
    }
}
