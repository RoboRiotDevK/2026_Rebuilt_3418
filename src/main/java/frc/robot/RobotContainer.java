// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoOrientCmd;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  CommandJoystick m_primary = Constants.OperatorConstants.PRIMARY;
  CommandXboxController m_secondary = Constants.OperatorConstants.SECONDARY;

  // Driver speeds

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  private final ShooterSubsystem shooter = new ShooterSubsystem(drivebase);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  public final DoubleSupplier getPosTwist = () -> m_primary.getRawAxis(5) * -1;
  private final DoubleSupplier aprilTag = () -> {
    if (shooter.overrideDrive) return shooter.aprilTagPos.getAsDouble();
    return getPosTwist.getAsDouble();
  };
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_primary.getY() * ((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0)),
      () -> m_primary.getX() * ((m_primary.getZ() - (23.0 / 9.0)) / (40.0 / 9.0)))
      .withControllerRotationAxis(aprilTag)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);
  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  public DoubleSupplier getNegTwist = () -> m_primary.getTwist();
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_primary::getTwist, getNegTwist)// checkfunction
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // DRIVETRAIN COMMAND ASSIGNMENTS R
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    final ChassisSpeeds DEATH_SPEEDS =  drivebase.getDeath();
    //for others reviewing, the DEATH_SPEEDS variable at line 95 has been tested and is safe for robot use
    //drive team is aware of this
    // create triggers for primary buttons
    // if joystick doesn't have the button you need
    BooleanSupplier zeroGyro = () -> m_primary.getHID().getRawButton(2);
    Trigger zeroGyroTrig = new Trigger(zeroGyro);
    BooleanSupplier deathMode = () -> m_primary.getHID().getRawButton(10);
    Trigger deathModeTrig = new Trigger(deathMode);

    // Auto Orient (I dont believe we need this - Darwin )
    m_primary.axisGreaterThan(6, .5).whileTrue(new AutoOrientCmd(drivebase, Constants.LIMELIGHT_PIPELINE_ID, 4.25, -3.9, 2));
    // Auto Commands

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // COMMAND/TRIGGER ASSIGNMENTS
    m_primary.button(1).onChange(shooter.ToggleOverride()).whileTrue(shooter.Shoot());


    // Primary Driver
    deathModeTrig.whileTrue(drivebase.driveCmd(DEATH_SPEEDS));
    // fullStopTrig.whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}