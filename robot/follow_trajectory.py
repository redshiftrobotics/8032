import pathfinder as pf

import wpilib
from wpilib import drive
from wpilib import ADXRS450_Gyro


class TrajectoryFollower:
    """
    Move along generated paths for autonomous
    """
    WHEEL_DIAMETER = 0.1524
    DRIVE_WIDTH = 1.0
    KV = 5.99
    KA = 0.717
    KS = 1.08
    DT = 0.02

    trajectories = {
        "charge": [
            pf.Waypoint(0, 0, 0),
            pf.Waypoint(1.5, 0, 0)
        ],
        "turn": [
            pf.Waypoint(0, 0, 0),
            pf.Waypoint(1, 1, pf.d2r(90))
        ]
    }

    def __init__(self, l_encoder, r_encoder):
        self._current_trajectory = None
        self.last_difference = 0

        self.l_encoder = l_encoder
        self.r_encoder = r_encoder

        self.left_follower = pf.followers.EncoderFollower(None)
        self.right_follower = pf.followers.EncoderFollower(None)

        self.left_follower.configurePIDVA(1.0, 0, 0, 1/self.KV, 0)
        self.right_follower.configurePIDVA(1.0, 0, 0, 1/self.KV, 0)

        self.cofigure_encoders()

        self.gyro = ADXRS450_Gyro()

    def follow_trajectory(self, trajectory_name: str):
        """
        Follow a specified trajectory.
        :param trajectory_name: The name of the trajectory to follow.
        """
        
        print('Following Trajectory:', trajectory_name)
        self._current_trajectory = trajectory_name
        trajectory = pf.generator.generate_trajectory(self.trajectories[trajectory_name], pf.hermite.pf_fit_hermite_cubic, pf.SAMPLES_FAST, self.DT, self.KV, self.KA, 120)
        l_trajectory, r_trajectory = pf.modifiers.tank(trajectory, self.DRIVE_WIDTH)
        self.left_follower.setTrajectory(l_trajectory)
        self.right_follower.setTrajectory(r_trajectory)

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
        return ((not self.left_follower.isFinished()) and (not self.right_follower.isFinished())) and self._current_trajectory == trajectory_name

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
            -self.gyro.getAngle()
        )  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(
            self.left_follower.getHeading()
        )  # Should also be in degrees

        # This is a poor man's P controller
        angle_difference = pf.bound_angle(desired_heading - gyro_heading)
        turn = self.KS * (-1.0 / 80.0) * angle_difference

        self.last_difference = angle_difference

        left += turn
        right -= turn

        print('Drive:', left, right)
        print('Encoders:', self.l_encoder.get(), self.r_encoder.get())

        return left, right
