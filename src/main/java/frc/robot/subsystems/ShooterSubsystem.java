package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.util.drivers.LimelightHelpers;
import frc.robot.util.drivers.LimelightHelpers.RawFiducial;
import frc.robot.util.math.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;
    CommandJoystick m_primary = Constants.OperatorConstants.PRIMARY;
    CommandXboxController m_secondary = Constants.OperatorConstants.SECONDARY;

    public double p, i, d;
    public PIDController pidController;
    public boolean overrideDrive = false;

    public ShooterSubsystem(SwerveSubsystem drivebase) {
        Instance = this;

        Log("Shooter subsystem loading...");
        Log("P: " + p + ", I: " + i + ", D: " + d);
        pidController = new PIDController(p, i, d);
        pidController.setSetpoint(1);

        LimelightHelpers.setPipelineIndex("limelight", Constants.LIMELIGHT_PIPELINE_ID);
    }

    /**
     * April tag position at hub (if seen)
     * @return limelight horizontal offset to april tag at hub, clamped between -0.8 and 0.8
     */
    public DoubleSupplier aprilTagPos = () -> {
        if (!LimelightHelpers.getTV("limelight")) return 0;
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
        if (!LimelightHelpers.getTV("limelight")) return 0.85; // set flywheel speed regardless of vision
        for (RawFiducial target : LimelightHelpers.getRawFiducials("limelight")) {
            if (target.id == 10 || target.id == 25) {
                return MathUtils.clamp((1 / Math.max(target.ta, 0.05)), 0.05, 1); // use math.max to avoid divide by zero errors
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

    /**
     * Command to shoot balls
     */
    public Command Shoot() {
        return run(() -> {
            // motor.set(pid.calculate(encoder.getDistance(), limelightCalculator())); this is pretty much everything
            // feed balls here
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
     * Log to console only in test mode
     * @param objects objects to log
     */
    public void Log(Object objects) {
        if (DriverStation.isTestEnabled()) {
            System.out.println(objects);
        }
    }
}
