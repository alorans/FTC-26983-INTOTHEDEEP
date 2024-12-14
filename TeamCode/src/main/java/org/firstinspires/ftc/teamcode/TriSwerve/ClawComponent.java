package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.command.AbstractComponent;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.Servo;

public class ClawComponent extends AbstractComponent {
    public final Servo wrist, hand;

    public ClawComponent(Servo wrist, Servo hand) {
        this.wrist = wrist;
        this.hand = hand;

        // Register subcomponents
        subcomponents.add(this.wrist);
        subcomponents.add(this.hand);
    }

    // Update super (which updates subcomponents)
    @Override
    public void update() {
        super.update();
    }

    // Set to setpoint
    public void set(Angle clawAngle, Angle wristAngle) {
        hand.setAngle(clawAngle);
        wrist.setAngle(wristAngle);
    }
}
