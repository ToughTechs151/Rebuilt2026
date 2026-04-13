package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/** The Game class contains functions specific to the game. */
public class Game {

  private RobotContainer robotContainer;
  private SwerveSubsystem drivebase;

  // **Pose for red alliance to the hub in meters and degrees. */
  public static final Pose2d RED_HUB_CENTER =
      new Pose2d(new Translation2d(11.552, 4.035), new Rotation2d());

  /** Pose for blue alliance to the hub in meters and degrees. */
  public static final Pose2d BLUE_HUB_CENTER =
      new Pose2d(new Translation2d(4.626, 4.035), new Rotation2d());

  // pose for the blue alliance left trench exit in meters and degrees (alliance side)
  public static final Pose2d BLUE_TRENCH_LEFT_EXIT =
      new Pose2d(new Translation2d(3.1, 7.376), Rotation2d.fromDegrees(180));
  // pose for the blue alliance right trench exit in meters and degrees (alliance side)
  public static final Pose2d BLUE_TRENCH_RIGHT_EXIT =
      new Pose2d(new Translation2d(3.1, 0.592), Rotation2d.fromDegrees(180));
  // pose for the blue alliance right trench approach/entrance in meters and degrees(neutral side)
  public static final Pose2d BLUE_TRENCH_RIGHT_APPROACH =
      new Pose2d(new Translation2d(6.000, 0.592), Rotation2d.fromDegrees(180));
  // pose for the blue alliance left trench approach/entrance in meters and degrees (neutral side)
  public static final Pose2d BLUE_TRENCH_LEFT_APPROACH =
      new Pose2d(new Translation2d(6.000, 7.376), Rotation2d.fromDegrees(180));
  // the y coordinate of the midline of the field
  public static final double FIELD_MIDLINE_Y = FlippingUtil.fieldSizeY / 2.0;
  // pose for the blue alliance left bump exit in meters and degrees (alliance side)
  public static final Pose2d BLUE_BUMP_LEFT_EXIT =
      new Pose2d(new Translation2d(2.995, 5.611), Rotation2d.fromDegrees(180));
  // pose for the blue alliance right bump exit in meters and degrees (alliance side)
  public static final Pose2d BLUE_BUMP_RIGHT_EXIT =
      new Pose2d(new Translation2d(2.995, 2.502), Rotation2d.fromDegrees(180));
  // pose for the blue alliance left bump approach/entrance in meters and degrees (neutral side)
  public static final Pose2d BLUE_BUMP_LEFT_APPROACH =
      new Pose2d(new Translation2d(5.995, 5.611), Rotation2d.fromDegrees(180));
  // pose for the blue alliance right bump approach/entrance in meters and degrees(neutral side)
  public static final Pose2d BLUE_BUMP_RIGHT_APPROACH =
      new Pose2d(new Translation2d(5.995, 2.502), Rotation2d.fromDegrees(180));

  // constants  for being at the hub and aimed towards it
  private static final double HUB_HEADING_TOL_DEG = 2.5;
  private static final double HUB_MIN_RADIUS_M = Units.feetToMeters(4.0);
  private static final double HUB_MAX_RADIUS_M = Units.feetToMeters(10.0);

  // Offsets for robot when launching and approaching
  private static final double LAUNCH_OFFSET = Units.feetToMeters(4.5);
  private static final double APPROACH_OFFSET = Units.feetToMeters(5.5);

  /** Constructor for the Game class. */
  public Game(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  /** Initialize the game class by setting up subsystems. */
  public void init() {
    this.drivebase = robotContainer.getDriveSubsystem();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /** Determine which trench to pass through. */
  public boolean isLeftTrench() {
    if (isRedAlliance()) {
      return drivebase.getPose().getY() < FIELD_MIDLINE_Y;
    } else {
      return drivebase.getPose().getY() > FIELD_MIDLINE_Y;
    }
  }

  /**
   * Calculate the distance to the alliance hub.
   *
   * @return The distance to the alliance hub in meters.
   */
  public double getDistanceToHub() {
    Pose2d hubPos = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    return drivebase.getPose().getTranslation().getDistance(hubPos.getTranslation());
  }

  /**
   * Calculate the angle to the alliance hub.
   *
   * @return The angle to the alliance hub as Rotation2d.
   */
  public Rotation2d getAngleToHub() {
    Pose2d hubPos = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    return hubPos.getTranslation().minus(drivebase.getPose().getTranslation()).getAngle();
  }

  /** Determines which hub center pose to use based on alliance color. */
  private Pose2d getHubCenterPose() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return RED_HUB_CENTER;
    }
    return BLUE_HUB_CENTER;
  }

  /** determine which trench exit pose to use */
  private Pose2d getTrenchExitPose() {
    if (isLeftTrench()) {
      return BLUE_TRENCH_LEFT_EXIT;
    } else {
      return BLUE_TRENCH_RIGHT_EXIT;
    }
  }

  /** determine which trench approach pose to use */
  private Pose2d getTrenchApproachPose() {
    if (isLeftTrench()) {
      return BLUE_TRENCH_LEFT_APPROACH;
    } else {
      return BLUE_TRENCH_RIGHT_APPROACH;
    }
  }

  /** determine which bump exit pose to use. */
  public Pose2d getBumpExitPose() {
    if (isLeftTrench()) {
      return BLUE_BUMP_LEFT_EXIT;
    } else {
      return BLUE_BUMP_RIGHT_EXIT;
    }
  }

  /** determine which bump approach pose to use. */
  public Pose2d getBumpApproachPose() {
    if (isLeftTrench()) {
      return BLUE_BUMP_LEFT_APPROACH;
    } else {
      return BLUE_BUMP_RIGHT_APPROACH;
    }
  }

  /**
   * Returns true if the robot is within the defined distance from the hub and aimed towards the
   * hub.
   */
  public boolean isRobotReadyAtHub() {
    Pose2d robotPose = drivebase.getPose();
    Pose2d hubPose = getHubCenterPose();

    if (!isRobotOnAllianceSideOfHub(robotPose, hubPose)) {
      return false;
    }

    Translation2d hubToRobot = robotPose.getTranslation().minus(hubPose.getTranslation());
    double dist = hubToRobot.getNorm();
    if (dist < HUB_MIN_RADIUS_M || dist > HUB_MAX_RADIUS_M) {
      return false;
    }
    Rotation2d desiredHeading = hubToRobot.getAngle(); // direction robot should face
    Rotation2d currentHeading = robotPose.getRotation(); // robot’s current yaw

    double headingErrorDeg = Math.abs(desiredHeading.minus(currentHeading).getDegrees());
    return headingErrorDeg <= HUB_HEADING_TOL_DEG;
  }

  // returns true if the robot is on the alliance side of the hub, false otherwise

  private boolean isRobotOnAllianceSideOfHub(Pose2d robotPose, Pose2d hubPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    double robotX = robotPose.getX();
    double hubX = hubPose.getX();

    if (alliance.get() == Alliance.Blue) {
      return robotX < hubX;
    } else {
      return robotX > hubX;
    }
  }

  // calculates the vector from the hub to the robot

  public Translation2d getHubToRobotVector() {
    Pose2d robotPose = drivebase.getPose();
    return robotPose.getTranslation().minus(getHubCenterPose().getTranslation());
  }

  // calculates the angle from the hub to the robot

  public Rotation2d getHubToRobotAngle() {
    return getHubToRobotVector().getAngle();
  }

  /** Command to drive with the launcher aimed at the alliance hub. */
  public Command aimHubDriveCommand(Supplier<ChassisSpeeds> velocity) {
    Pose2d hubTarget = isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    return drivebase.aimAtPoseCommand(velocity, hubTarget);
  }

  /** Periodic function to update SmartDashboard values. */
  public void periodic() {
    SmartDashboard.putNumber("Hub/Distance", getDistanceToHub());
    SmartDashboard.putNumber("Hub/Angle", getAngleToHub().getDegrees());
  }

  /** Command to drive the robot through the trench. */
  public Command driveTrenchCommand() {
    return Commands.defer(
        () -> {
          // poses for the trench approach and exit
          Pose2d approachPose = getTrenchApproachPose();
          Pose2d exitPose = getTrenchExitPose();
          List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(approachPose, exitPose);
          // create the path with constraints and starting/ending states
          PathPlannerPath driveTrench =
              new PathPlannerPath(
                  waypoints,
                  DriveConstants.DRIVE_TRENCH_CONSTRAINTS,
                  new IdealStartingState(1.0, Rotation2d.fromDegrees(0)),
                  new GoalEndState(1.0, Rotation2d.fromDegrees(0)));
          // return the command to drive to appraoch pose and
          // follow path from appraoch to trenchcenter
          return drivebase.driveAndFollowPath(driveTrench);
        },
        Set.of(drivebase));
  }

  /** Command to drive the robot through the bump. */
  public Command driveBumpCommand() {
    return Commands.defer(
        () -> {
          // poses for the bump approach and exit
          Pose2d approachPose = getBumpApproachPose();
          Pose2d exitPose = getBumpExitPose();
          // create a list of the points to pass through
          List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(approachPose, exitPose);
          // create the path with constraints and starting/ending states
          PathPlannerPath driveBump =
              new PathPlannerPath(
                  waypoints,
                  DriveConstants.DRIVE_BUMP_CONSTRAINTS,
                  new IdealStartingState(1.0, Rotation2d.fromDegrees(45)),
                  new GoalEndState(1.0, Rotation2d.fromDegrees(45)));
          // return the command to drive to approach pose and follow path
          // from approach to bump center
          return drivebase.driveAndFollowPath(driveBump);
        },
        Set.of(drivebase));
  }

  /** Creates a command to drive to the nearest hub and aim for launching. */
  public Command driveHubCommand() {
    // Ensures everything runs at run time, instead of after
    return Commands.defer(
        () -> {
          // Pose of robot at hub
          Pose2d hubCenter = getHubCenterPose();
          Rotation2d hubAngle = getHubToRobotAngle();
          // Pose of robot at trench

          // Limit the target angle to a range in the alliance zone
          double angleToHub;
          if (drivebase.isRedAlliance()) {
            angleToHub = MathUtil.clamp(hubAngle.getDegrees(), -35, 35);
          } else {
            angleToHub =
                MathUtil.clamp(hubAngle.minus(Rotation2d.fromDegrees(180)).getDegrees(), -35, 35)
                    + 180.0;
          }
          hubAngle = Rotation2d.fromDegrees(angleToHub);

          // Movement for robot to shooting and approach location
          Translation2d launchTranslation = new Translation2d(LAUNCH_OFFSET, hubAngle);
          Translation2d approachTranslation = new Translation2d(APPROACH_OFFSET, hubAngle);

          // Actual robot positions
          Pose2d launchPose =
              new Pose2d(hubCenter.getTranslation().plus(launchTranslation), hubAngle);
          Pose2d approachPose =
              new Pose2d(hubCenter.getTranslation().plus(approachTranslation), hubAngle);

          // Return command to drive
          return drivebase.driveToPosePID(approachPose, launchPose);
        },
        Set.of(drivebase));
  }
}
