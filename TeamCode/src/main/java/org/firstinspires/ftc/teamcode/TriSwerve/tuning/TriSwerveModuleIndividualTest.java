package org.firstinspires.ftc.teamcode.TriSwerve.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TriSwerve.TriRobot;
import org.firstinspires.ftc.teamcode.TriSwerve.TriSwerveModule;

import java.util.List;
import java.util.stream.Collectors;

@TeleOp(group = "TriSwerve")
public class TriSwerveModuleIndividualTest extends CommandOpMode {
    @Register
    private TriRobot robot;

    // TriSwerve devices
    private TriSwerveModule triSwerve = null;
    private Component triSwerveComponent = null;
    private int moduleIndex = 0;

    @Override
    public void preInit() {
        // Unregister robot drive
        unregister(robot.drive);

        // Get tri-swerve modules list
        final List<SwerveModule> swerveModules = robot.drive.getModules();
        assert swerveModules.stream().allMatch(item -> item instanceof TriSwerveModule) : "Not all items are instances of Child";
        List<TriSwerveModule> triSwerveModules = swerveModules.stream()
                .map(item -> (TriSwerveModule) item) // Safe cast since assert
                .collect(Collectors.toList());
        triSwerveModules.forEach(item -> item.setModuleOrientation(Angle.rad(Math.PI / 4)));

        // Register current triSwerve components
        triSwerve = triSwerveModules.get(moduleIndex);
        triSwerveComponent = Component.of(triSwerve::update);
        register(triSwerveComponent);

        // Map X button
        map(gamepad().p1.x::isJustActivated, () -> {
            // Unregister previous swerve module
            unregister(triSwerveComponent);
            triSwerve.servo.setPower(0.0);

            // Get new swerve module
            moduleIndex = ++moduleIndex % 3;
            triSwerve = triSwerveModules.get(moduleIndex);

            // Initialize and register new module
            triSwerve.setModuleOrientation(Angle.rad(Math.PI / 4));
            triSwerveComponent = Component.of(triSwerve::update);
            register(triSwerveComponent);
        });

        // Map A and B buttons to set module orientation
        map(gamepad().p1.a::isJustActivated, () ->
            triSwerve.setModuleOrientation(Angle.rad(Math.PI / 4))
        );
        map(gamepad().p1.b::isJustActivated, () ->
            triSwerve.setModuleOrientation(Angle.rad(7 * Math.PI / 4))
        );

        // Print servo speed on telemetry
        schedule(true, () -> {
            telem.addData("Measured servo power", triSwerve.servo.getPower());
            telem.addData("Encoder angle", triSwerve.moduleOrientationSensor.getAngle());
        });
    }
}