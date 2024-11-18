"""The Flip Node."""
from geometry_msgs.msg import Twist, Vector3

import rclpy
from rclpy.node import Node


def turtle_twist(linear_velocity, angular_velocity):
    """
    Create a twist velocity suitable for a turtle.

    Args:
    linear_vel (list of floats): the linear velocities
    angular_vel (list of floats): the angular velocities

    Returns
    -------
    Twist: a 2D twist object corresponding to linear/angular velocity

    """
    return Twist(
        linear=Vector3(
            x=linear_velocity[0], y=linear_velocity[1], z=linear_velocity[2]
        ),
        angular=Vector3(
            x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2]
        ),
    )


class Flip(Node):
    """The Flip Node."""

    def __init__(self):
        """Initialise timer, publisher and helper variables."""
        super().__init__('flip')

        self.vel_tmr = self.create_timer(1, self.vel_tmr_callback)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.counter_var = 0
        self.sign_flag = 1
        self.pause = False
        self.pause_cntr = 0

    def vel_tmr_callback(self):
        """Timer publishing cmd_vel commands at 1Hz."""
        if self.pause is True:
            stop_vel = 0.0
            stop_twist = turtle_twist([stop_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.cmd_vel_publisher.publish(stop_twist)
            self.pause_cntr += 1
            if (self.pause_cntr == 4):
                self.pause = False
                self.pause_cntr = 0
        else:
            x_vel = self.sign_flag * 12.0
            forward_twist = turtle_twist([x_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.cmd_vel_publisher.publish(forward_twist)
            self.counter_var += 1

        if (self.counter_var == 3):
            self.pause = True
            self.counter_var = 0
            self.sign_flag = -1 * self.sign_flag


def main(args=None):
    """Spin the flip node."""
    rclpy.init(args=args)
    flip = Flip()
    rclpy.spin(flip)
    flip.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
