package org.firstinspires.ftc.teamcode.OtherSubsystems;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.control.DCMotorFeedforward;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;
@JoosConfig
public class Slide implements Component {
    // Objects
    private final HardwareMap hardwareMap;
    private final DcMotor slideMotor;
    private  double currentPos;
    private double targetPos;
    private double slideSpeed;
    private DoubleSupplier controlRY;



    // Default constructor
    public Slide(HardwareMap hardware, String slideMotorId, DoubleSupplier controlRY){
        this.hardwareMap = hardware;
        this.slideMotor = hardwareMap.dcMotor.get(slideMotorId);
        slideSpeed=0;
        this.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPos=slideMotor.getCurrentPosition();

        this.controlRY = controlRY;
    }



    // Always maintain pos
    @Override
    public void update(){
        if (Math.abs(-controlRY.getAsDouble()) > 0.05) {
            // Move the motor based on joystick input
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(targetPos<5000){
                slideMotor.setPower(-controlRY.getAsDouble());
            }
            else if(-controlRY.getAsDouble()<0){
                slideMotor.setPower(-controlRY.getAsDouble());
            }
            else slideMotor.setPower(0);

            // Update the target position for holding
            targetPos = slideMotor.getCurrentPosition();

        } else {
            slideMotor.setTargetPosition((int) targetPos);
            // Hold position
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor.setPower(0.5); // Adjust power as needed for holding
        }


    }

    // FTC-dashboard debug functions
    public double getCurrentPos() {
        return slideMotor.getCurrentPosition();
    }
}


