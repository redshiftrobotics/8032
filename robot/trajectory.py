import pathfinder as pf
import wpilib
from wpilib import drive
from ctre.wpi_talonsrx import WPI_TalonSRX


class TrajectoryFollower:
    """
    Move along generated paths for autonomous
    """
    WHEEL_DIAMETER = 0.1524
    KV = 5.99
    KA = 0.717
    KS = 1.08

    def __init(self, l_encoder, r_encoder):
        self._current_trajectory = None
        self.last_difference = 0

        self.l_enocder = l_encoder
        self.r_encoder = r_encoder

        self.left_follower = pf.followers.EncoderFollower(None)
        self.right_follower = pf.followers.EncoderFollower(None)

        self.left_follower.configurePIDVA(1.0, 0, 0, self.KV, self.KA)
        self.right_follower.configurePIDVA(1.0, 0, 0, self.KV, self.KA)

        self.cofigure_encoders()

    def follow_trajectory(self, trajectory_name: str):
        """
        Follow a specified trajectory.
        :param trajectory_name: The name of the trajectory to follow.
        """
        
        print('Following Trajectory:', trajectory_name)
        self._current_trajectory = trajectory_name
        self.left_follower.setTrajectory(self.generated_trajectories[trajectory_name][0])
        self.right_follower.setTrajectory(self.generated_trajectories[trajectory_name][1])

        self.cofigure_encoders()

    def cofigure_encoders(self):
        """
        Configure the encoders for following a trajectory.
        """
        # Resets Encoders
        self.l_encoder.setSelectedSensorPosition(0, 0, 10)
        self.r_encoder.setSelectedSensorPosition(0, 0, 10)

        # Configures them
        self.left_follower.configureEncoder(self.l_encoder.getSelectedSensorPosition(0), 4096, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(self.r_encoder.getSelectedSensorPosition(0), 4096, self.WHEEL_DIAMETER)

    def is_following(self, trajectory_name):
        """
        Check whether a trajectory is being followed.
        :param trajectory_name: The name of the trajectory to check.
        """
        return self._current_trajectory is not None and self._current_trajectory == trajectory_name

    def run(self):
        """
        Calculate the movement values and move the robot.
        """
        if (self.left_follower.trajectory is None or self.right_follower.trajectory is None) or \
           (self.left_follower.isFinished() and self.right_follower.isFinished()):
            self._current_trajectory = None
            return

        left = self.left_follower.calculate(self.l_encoder.get())
        right = self.right_follower.calculate(self.r_encoder.get())

        gyro_heading = (
            -self.navx.getAngle()
        )  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(
            self.left_follower.getHeading()
        )  # Should also be in degrees

        # This is a poor man's P controller
        angle_difference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        # turn = (1.1 * (-1.0 / 80.0) * angle_difference) + (0.05 * (angle_difference - self.last_difference))
        turn = self.ANGLE_CONSTANT * (-1.0 / 80.0) * angle_difference
        # turn = 0

        self.last_difference = angle_difference

        left += turn
        right -= turn

        print('Drive:', left, right)
        print('Encoders:', self.l_encoder.get(), self.r_encoder.get())

        return left, right
