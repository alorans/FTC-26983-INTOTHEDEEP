package org.firstinspires.ftc.teamcode.TriSwerve.tuning;

import com.amarcolini.joos.command.BasicCommand;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.InstantCommand;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TriSwerve.TriRobot;
import org.firstinspires.ftc.teamcode.TriSwerve.TriSwerveModule;

import java.util.List;
import java.util.stream.Collectors;

@TeleOp(group = "TriSwerve")
public class TriSwerveModuleCurrentTest extends CommandOpMode {
    @Register
    private TriRobot robot;

    // TriSwerve devices
    private TriSwerveModule triSwerve = null;
    private int moduleIndex = 0;
    private double direction = 1.0;

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

        // Map X button
        map(gamepad().p1.x::isJustActivated, () -> {
            // Get new swerve module
            moduleIndex = ++moduleIndex % 3;
            triSwerve = triSwerveModules.get(moduleIndex);
        });

        // Map A and B buttons to set module orientation
        SequentialCommand spin = new SequentialCommand(
                false,
                /*new BasicCommand(() -> {
                    telem.addData("direction", direction);
                    telem.addData("start_time", getRuntime());
                }),*/
                new TimeCommand((t, dt) -> {
                    triSwerve.servo.setPower(direction);
                    return t >= 1;
                }).onEnd((over) -> triSwerve.servo.setPower(0.0))
        );
        map(gamepad().p1.a::isJustActivated, () -> {
            direction = 1.0;
            spin.schedule();
        });
        map(gamepad().p1.b::isJustActivated, () -> {
            direction = -1.0;
            spin.schedule();
        });
    }
}