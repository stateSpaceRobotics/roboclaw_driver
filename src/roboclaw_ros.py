
import roboclaw_driver

class RoboclawROS:

    def __init__(
        self,
        name
    ):
        # Store name for use in accessing ROS parameters
        self.name = name
        self.__create_roboclaw()
        self.__load_params()

    ADDR = 1
    CHANNEL = 1
    QPPS = 2
    CTRL = 3
    SUB = 4
    PUB = 5

    def __create_roboclaw(self):
        # Create the roboclaw and load parameters for the serial port
        self._roboclaw_interface = roboclaw_driver.Roboclaw(
            rospy.get_param("~/{}/serial_port".format(self.name), "/dev/ttyUSB0"),
            rospy.get_param("~/{}/serial_baud".format(self.name), 460800),
            rospy.get_param("~/{}/serial_timeout".format(self.name), 0.01),
            rospy.get_param("~/{}/serial_retries".format(self.name), 3)
        )

    def __load_params(self):
        # Load ROS parameters

        # Load list of Roboclaw addresses and channel names
        self._roboclaw_addresses = rospy.get_param("~/{}/addresses".format(self.name), [])

        # Load roboclaw channel names
        self._channel_dict
        # This dict has a list of channel-specific objects:
        # [address, QPPS, control_type, Subscriber, Publisher]
        for addr in self._roboclaw_addresses:
            # Get the M1 channel's name
            M1_name = rospy.get_param("~/{}/{}/M1/name".format(self.name, addr), "{}/M1".format(addr))
            # Create an entry in the channel dict for it
            self._channel_dict[M1_name] = []
            # Add the device address first
            self._channel_dict[M1_name].append(addr)
            # Add the channel side (M1/M2)
            self._channel_dict[M1_name].append(True)
            # Add QPPS
            self._channel_dict[M1_name].append(
                rospy.get_param("~/{}/{}/M1/QPPS".format(self.name, addr), 1)
            )
            # Add its control type flag (True for velocity, False for position)
            self._channel_dict[M1_name].append(
                rospy.get_param("~/{}/{}/M1/control_type".format(self.name, addr), True)
            )
            self._channel_dict[M1_name].append(
                self.__create_callback(M1_name)
            )

            # Get the M1 channel's name
            M2_name = rospy.get_param("~/{}/{}/M2/name".format(self.name, addr), "{}/M2".format(addr))
            # Create an entry in the channel dict for it
            self._channel_dict[M2_name] = []
            # Add the device address first
            self._channel_dict[M2_name].append(addr)
            # Add the channel side (M1/M2)
            self._channel_dict[M1_name].append(False)
            # Add QPPS
            self._channel_dict[M2_name].append(
                rospy.get_param("~/{}/{}/M2/QPPS".format(self.name, addr), 1)
            )
            # Add its control type flag (True for velocity, False for position)
            self._channel_dict[M2_name].append(
                rospy.get_param("~/{}/{}/M2/control_type".format(self.name, addr), True)
            )
            self._channel_dict[M2_name].append(
                self.__create_callback(M2_name)
            )

    def __create_callback(self,name):
        
        if self._channel_dict[name][self.CTRL]:
            # create a velocity callback and subscriber
            self._channel_dict[name].append(
                rospy.Subscriber("{}/setVel".format(name), std_msgs.msg.Float64, lambda x: self.velocity_callback(name, x))
            )
            # create velocity publisher
            self._channel_dict[name].append(
                rospy.Publisher("{}/getVel".format(name), std_msgs.msg.Float64)
            )

        else:
            # create a velocity callback and subscriber
            self._channel_dict[name].append(
                rospy.Subscriber("{}/setPos".format(name), std_msgs.msg.Float64, lambda x: self.position_callback(name, x))
            )
            # create velocity publisher
            self._channel_dict[name].append(
                rospy.Publisher("{}/getPos".format(name), std_msgs.msg.Float64)
            )

    def velocity_callback(self, name, msg):
        if self._channel_dict[name][self.CHANNEL]:
            success = self._roboclaw_interface.SpeedAccelDistanceM1(
                self._channel_dict[name][self.ADDR],
                accel=1, # TODO: Parameterize
                speed=msg.data,
                distance=msg.data*2, # TODO: Parameterize
                buffer=1
            )

        else:
            success = self._roboclaw_interface.SpeedAccelDistanceM2(
                self._channel_dict[name][self.ADDR],
                accel=1, # TODO: Parameterize
                speed=msg.data,
                distance=msg.data*2, # TODO: Parameterize
                buffer=1
            )

        # TODO: Verify message successfully sent

    def publish_update(self):
        # For each channel
        for name in self._channel_dict.keys():
            # if it's velocity
            if self._channel_dict[name][self.CTRL]:
                # M1 channels
                if self._channel_dict[name][self.CHANNEL]:
                    resp = self._roboclaw_interface.ReadSpeedM1(
                        self._channel_dict[name][self.ADDR]
                    )
                # M2 channels
                else:
                    resp = self._roboclaw_interface.ReadSpeedM2(
                        self._channel_dict[name][self.ADDR]
                    )

                # create msg
                msg = std_msgs.msg.Float64()
                msg.data = float(resp)
                self._channel_dict[name][self.PUB].publish(msg)
            
    def update(self):
        self.publish_update()