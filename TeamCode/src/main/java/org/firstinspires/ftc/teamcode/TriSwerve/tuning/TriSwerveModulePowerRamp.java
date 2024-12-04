package org.firstinspires.ftc.teamcode.TriSwerve.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.util.NanoClock;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TriSwerve.TriRobot;
import org.firstinspires.ftc.teamcode.TriSwerve.TriSwerveModule;

import java.util.List;
import java.util.stream.Collectors;

@TeleOp(group = "TriSwerve")
public class TriSwerveModulePowerRamp extends CommandOpMode {
    @Register
    private TriRobot robot;

    // TriSwerve devices
    private TriSwerveModule triSwerve = null;
    private int moduleIndex = 0;
    private double power = 0.0;
    private boolean running = true;
    // Angle velocity variables
    private Angle lastAngle = Angle.deg(0.0);
    private double lastUpdate = 0.0;
    private NanoClock clock = NanoClock.getSystem();

    // Utility for clamping power
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void preInit() {

        // Unregister robot drive
        unregister(robot.drive);

        // Get tri-swerve modules list
        final List<SwerveModule> swerveModules = robot.drive.getModules();
        assert swerveModules.stream().allMatch(item -> item instanceof TriSwerveModule) : "Not all items are instances of Child";
        List<TriSwerveModule> triSwerveModules = swerveModules.stream()
                .map(item -> (TriSwerveModule) item) // Safe cast since assert
                .collect(Collectors.toList());       // Use collectors to turn into list
        triSwerveModules.forEach(item -> item.setModuleOrientation(Angle.rad(Math.PI / 4)));

        // Register current triSwerve components
        triSwerve = triSwerveModules.get(moduleIndex);
        lastAngle = triSwerve.moduleOrientationSensor.getAngle();
        lastUpdate = clock.seconds();

        // Create swerve servo component with update method
        Component.of(() -> {
            // Handle power
            if (running) triSwerve.servo.setPower(power);
            else triSwerve.servo.setPower(0.0);
            telem.addData("power", power);

            // Calculate angular velocity (degrees per second)
            Angle currentAngle = triSwerve.moduleOrientationSensor.getAngle();
            telem.addData("angle", currentAngle);
            Angle angleDiff = currentAngle.minus(lastAngle);
            double dt = clock.seconds() - lastUpdate;

            Angle angleVel = angleDiff.div(dt);
            telem.addData("angular_vel", angleVel);

            lastAngle = triSwerve.moduleOrientationSensor.getAngle();
            lastUpdate = clock.seconds();
        }).register();

        // Map X button to change swerve pod
        map(gamepad().p1.x::isJustActivated, () -> {
            // Get new swerve module
            triSwerve.servo.setPower(0.0);
            moduleIndex = ++moduleIndex % 3;
            triSwerve = triSwerveModules.get(moduleIndex);
        });

        // Map other buttons to control power
        map(gamepad().p1.y::isJustActivated, () -> running = !running);
        map(gamepad().p1.a::isJustActivated, () -> power = clamp(power + 0.025, -1, 1));
        map(gamepad().p1.b::isJustActivated, () -> power = clamp(power - 0.025, -1, 1));
        map(gamepad().p1.dpad_up::isJustActivated, () -> power = clamp(power + 0.0025, -1, 1));
        map(gamepad().p1.dpad_down::isJustActivated, () -> power = clamp(power - 0.0025, -1, 1));
    }
}
