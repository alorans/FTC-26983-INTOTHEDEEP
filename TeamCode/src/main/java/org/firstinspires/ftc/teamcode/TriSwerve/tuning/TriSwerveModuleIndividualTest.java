package org.firstinspires.ftc.teamcode.TriSwerve.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TriSwerve.TriRobot;
import org.firstinspires.ftc.teamcode.TriSwerve.TriSwerveModule;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

@TeleOp(group = "TriSwerve")
public class TriSwerveModuleIndividualTest extends CommandOpMode {
    @Register
    private TriRobot robot;

    @Override
    public void preInit() {
        // Unregister robot drive
        unregister(robot.drive);

        // Get tri-swerve modules
        final List<SwerveModule> swerveModules = robot.drive.getModules();
        assert swerveModules.stream().allMatch(item -> item instanceof TriSwerveModule) : "Not all items are instances of Child";
        List<TriSwerveModule> triSwerveModules = swerveModules.stream()
                .map(item -> (TriSwerveModule) item) // Safe cast since we asserted
                .collect(Collectors.toList());
        triSwerveModules.forEach(item -> item.setModuleOrientation(Angle.rad(Math.PI / 4)));

        // Switch swerve module with X button
        AtomicReference<TriSwerveModule> triSwerve = new AtomicReference<>(triSwerveModules.get(0));
        AtomicReference<Component> triSwerveComponent = new AtomicReference<>(Component.of(triSwerve.get()::update));
        register(triSwerveComponent.get());
        AtomicInteger moduleIndex = new AtomicInteger();
        map(gamepad().p1.x::isJustActivated, () -> {
            unregister(triSwerveComponent.get());
            triSwerve.get().servo.setPower(0.0);
            moduleIndex.set(moduleIndex.incrementAndGet() % 3);
            triSwerve.set(triSwerveModules.get(moduleIndex.get()));
            triSwerve.get().getTargetOrientation();
            triSwerve.get().setModuleOrientation(Angle.rad(Math.PI / 4));
            triSwerveComponent.set(Component.of(triSwerve.get()::update));
            register(triSwerveComponent.get());
        });

        // Map A and B buttons to set module orientation
        map(gamepad().p1.a::isJustActivated, () -> {
            triSwerve.get().setModuleOrientation(Angle.rad(Math.PI / 4));
        });
        map(gamepad().p1.b::isJustActivated, () -> {
            triSwerve.get().setModuleOrientation(Angle.rad(7 * Math.PI / 4));
        });

        // Print servo speed on telemetry
        schedule(true, () -> {
            telem.addData("Measured servo power", triSwerve.get().servo.getPower());
        });
    }
}