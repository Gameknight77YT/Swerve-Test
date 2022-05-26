// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SwerveAutoCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final XboxController controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivetrain.setDefaultCommand(new DefaultDriveCommand(
            drivetrain,
            () -> -modifyAxis(controller.getLeftY())/3 * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getLeftX())/3 * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getRightX())/2 * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(controller::getRightBumper)
            // No requirements because we don't need to interrupt anything
            .whenPressed(drivetrain::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
        Constants.MAX_VELOCITY_METERS_PER_SECOND/2,
        Constants.MAX_VELOCITY_METERS_PER_SECOND/3)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(Constants.kinematics);

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these  interior waypoints
        List.of(new Translation2d(1, 0)),
        // End 
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(
      "Test",
      Constants.MAX_VELOCITY_METERS_PER_SECOND/3,
      Constants.MAX_VELOCITY_METERS_PER_SECOND/5);
    

    var thetaController =
      new ProfiledPIDController(
      Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      PPSwerveControllerCommand  swerveCommand = new PPSwerveControllerCommand(
        examplePath, 
        drivetrain::getPose, 
        Constants.kinematics, 
        new PIDController(Constants.kPXController, 0, 0), 
        new PIDController(Constants.kPYController, 0, 0), 
        thetaController, 
        drivetrain::setModuleStates, 
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveCommand
    .andThen(() -> drivetrain.stopDrive());

  }

  private static double deadband(double value, double deadband) { 
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
