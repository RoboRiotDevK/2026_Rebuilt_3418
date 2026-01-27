package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LimelightHelpers;
import frc.robot.util.drivers.LimelightHelpers.RawFiducial;
import frc.robot.util.math.MathUtils;

/** Shooter subsystem for controlling the flywheel(s) */

// TODO: Tune PID values
// TODO: Tune limelightcalc
// TODO: Test PID stuff
// TODO: Test limelight stuff

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;

    public double 
    p = 0.1,
    i = 0.01,
    d = 0;
    
    public PIDController pidController;
    /**
     * If true, override drive control with april tag position
     */
    public boolean overrideDrive = false;
    static boolean trigger = false;

    SparkMax sparkMaxA, sparkMaxB;
    AbsoluteEncoder encoderA, encoderB;

    public ShooterSubsystem() {
        Instance = this;

        Log("Shooter subsystem loading...");
        Log("P: " + p + ", I: " + i + ", D: " + d);
        /*pidController = new PIDController(p, i, d);
        pidController.setSetpoint(1);*/

        sparkMaxA = new SparkMax(16, SparkMax.MotorType.kBrushless);
        sparkMaxB = new SparkMax(24, SparkMax.MotorType.kBrushless);

        encoderA = sparkMaxA.getAbsoluteEncoder();
        encoderB = sparkMaxB.getAbsoluteEncoder();

        pidController = new PIDController(p, i, d);
        pidController.setSetpoint(0);
        //pidController.setTolerance(0.05, 0.05);

        LimelightHelpers.setPipelineIndex("limelight", Constants.LIMELIGHT_PIPELINE_ID);
    }

    /**
     * April tag position at hub (if seen)
     * @return limelight horizontal offset to april tag at hub, clamped between -0.8 and 0.8
     */
    public DoubleSupplier aprilTagPos = () -> {
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return 0;
        for (RawFiducial target : LimelightHelpers.getRawFiducials("limelight")) {
            if (target.id == 10 || target.id == 25) { // both tag ids at hub
                return MathUtils.clamp(target.txnc, -0.8, 0.8);
            }
        }

        return 0;
    };

    /**
     * Calculates flywheel speed based on limelight data. If no target, returns 0.85
     * @return flywheel speed (0.05 to 1)
     */
    public double limelightCalculator() {
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return 0.85; // set flywheel speed regardless of vision
        for (RawFiducial target : LimelightHelpers.getRawFiducials("limelight")) {
            if (target.id == 10 || target.id == 25) {
                return MathUtils.clamp((Math.max(target.distToCamera, 0.05)) / 10, 0.05, 1); // TUNE THIS PLEASE
            }
        }


        return 0.85;
    }
    /**
     * Toggle override for drive control
     */
    public Command ToggleOverride() {
        return runOnce(() -> {
            overrideDrive = !overrideDrive;
        });
    }

    public Command triggerThing() {return runOnce(() -> {trigger = !trigger; } ); }

    DoubleSupplier getSetpoint = () -> {
        if (trigger)
            return 0.7;
        else 
            return -0.2;
    };

    /**
     * Command to shoot balls
     */
    public Command Shoot() {
        return run(() -> {
            double beforeClamp = pidController.calculate(encoderA.getVelocity() / Constants.MAX_NEO_VORTEX_SPEED, getSetpoint.getAsDouble()) * 10;
            //System.out.println("Trigger: " + trigger + " ts: " + beforeClamp);

            double speed = MathUtils.clamp( beforeClamp, 0, 0.7);

            sparkMaxA.set(speed); // facing opposite dir
            sparkMaxB.set(speed); // facing same direction (as of 2026-01-24)
        });
    }

    /**
     * Debug command to update PID values
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public Command UpdatePID(double kP, double kI, double kD) {
        return runOnce(() -> {
            pidController.setPID(kP, kI, kD);
            p = kP;
            i = kI;
            d = kD;
        });
    }

    /**
     * Debug command to update PID values
     */
    public Command UpdatePID() {
        return runOnce(() -> {
            pidController.setPID(p, i, d);
        });
    }


    /**
     * Log to console only in test mode
     * @param objects objects to log
     */
    public void Log(Object objects) {
        if (DriverStation.isTestEnabled()) {
            System.out.println(objects);
        }
    }
}
