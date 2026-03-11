package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;

public class AlignToHub extends Command {
    private PIDController xController, yController;
    private Timer dontSeeTagTimer, stopTimer;
    private int tagID = -1;
    private final CANDriveSubsystem driveSubsystem;

    public AlignToHub(CANDriveSubsystem driveSubsystem) {
        xController = new PIDController(DriveConstants.X_ALIGNMENT_P, 0, 0);
        yController = new PIDController(DriveConstants.Y_ALIGNMENT_P, 0, 0);
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        tagID = -1;
        
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        xController.reset();
        yController.reset();

        xController.setSetpoint(DriveConstants.X_SETPOINT_ALIGNMENT);
        xController.setTolerance(DriveConstants.X_TOLERANCE_ALIGNMENT);

        yController.setSetpoint(DriveConstants.Y_SETPOINT_ALIGNMENT);
        yController.setTolerance(DriveConstants.Y_TOLERANCE_ALIGNMENT);

        if (LimelightHelpers.getTV("")) {
            tagID = (int) Math.round(LimelightHelpers.getFiducialID(""));
        }
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("");
        int currentTag = hasTarget ? (int) Math.round(LimelightHelpers.getFiducialID("")) : -1;

        if (hasTarget && tagID == -1) {
            tagID = currentTag;
        }

        if (hasTarget && currentTag == tagID) {
            this.dontSeeTagTimer.reset();

            double tx = LimelightHelpers.getTX("");
            double xSpeed = xController.calculate(tx);
            // double ySpeed = -yController.calculate(positions[2]);

            // if (Math.abs(yController.getError()) < DriveConstants.Y_TOLERANCE_ALIGNMENT) {
            //     driveSubsystem.arcadeDrive(ySpeed, xSpeed);
            // } else {
            //     driveSubsystem.arcadeDrive(0.0, xSpeed);
            // }
            
       

            if (xSpeed > 0 ) {
                xSpeed = Math.min(xSpeed, 0.5);
            } else {
                xSpeed = Math.max(xSpeed, -0.5);
            }

            driveSubsystem.arcadeDrive(0.0, xSpeed);

            if (!yController.atSetpoint() || !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            driveSubsystem.arcadeDrive(0.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return this.dontSeeTagTimer.hasElapsed(DriveConstants.DONT_SEE_TAG_WAIT_TIME)
                || stopTimer.hasElapsed(DriveConstants.POSE_VALIDATION_TIME);
    }
}
