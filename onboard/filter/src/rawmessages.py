import time
import math


class raw_imu:
    def __init__(self, num_prev_bearings=5):
        self._acc_x = None
        self._acc_y = None
        self._acc_z = None
        self._gyro_x = None
        self._gyro_y = None
        self._gyro_z = None
        self._mag_x = None
        self._mag_y = None
        self._mag_z = None
        self._bearing = None
        self._prev_bearings = []
        self._num_prev_bearings = num_prev_bearings
        self._time_of_IMU = time.clock()

    def update_imu_bearing(self, message):
        self._acc_x = message.accel_x
        self._acc_y = message.accel_y
        self._acc_z = message.accel_z
        self._gyro_x = message.gyro_x
        self._gyro_y = message.gyro_y
        self._gyro_z = message.gyro_z
        self._mag_x = message.mag_x
        self._mag_y = message.mag_y
        self._mag_z = message.mag_z
        self._bearing = message.bearing
        self._prev_bearings.append(self._bearing)
        if len(self._prev_bearings) > self._num_prev_bearings:
            self._prev_bearings.pop(0)
        self._time_of_IMU = time.clock()

        self.calc_pitch_roll_yaw()

    def linear_moving_avg(self):
        """
        Calculates a weighted linear moving average over the previous 5
        bearing readings.
        """
        if not self._prev_bearings:
            return None
        total_bearings = 0
        total_weights = 0
        for i in range(len(self._prev_bearings)):
            total_bearings += (i + 1) * self._prev_bearings[i]
            total_weights += (i + 1)
        return total_bearings / total_weights

    def calc_pitch_roll_yaw(self):
        """Calculates the bearing based on the magnetometer readings."""
        acc_yz = math.sqrt(math.pow(self._acc_y, 2) + math.pow(self._acc_z, 2))
        acc_xz = math.sqrt(math.pow(self._acc_x, 2) + math.pow(self._acc_z, 2))
        self._pitch = 180 * math.atan2(self._acc_x, acc_yz) / math.pi
        self._roll = 180 * math.atan2(self._acc_y, acc_xz) / math.pi
        cos_pitch = math.cos(self._pitch)
        sin_pitch = math.sin(self._pitch)
        cos_roll = math.cos(self._roll)
        sin_roll = math.sin(self._roll)
        partial_mag_xx = self._mag_x * cos_pitch
        partial_mag_xy = self._mag_y * sin_roll * sin_pitch
        partial_mag_xz = self._mag_z * cos_roll * sin_pitch
        mag_x = partial_mag_xx + partial_mag_xy + partial_mag_xz
        mag_y = self._mag_y * cos_roll - self._mag_z * sin_roll
        self._yaw = 180 * math.atan2(-mag_y, mag_x) / math.pi
        print('pitch: {}'.format(self._pitch))
        print('roll: {}'.format(self._roll))
        print('yaw: {}'.format(self._yaw))
        theta = math.atan2(self._mag_y, self._mag_x)  # radians
        self._mag_bearing = 90 - theta * 180 / math.pi
        print('manual magnetometer bearing: {}'.format(self._mag_bearing))


class raw_gps:
    def __init__(self):
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None
        self._track_angle = None
        self._ground_speed = None
        self._time_of_GPS = time.clock()

    def updateGPS(self, message):
        self._lat_deg = message.latitude_deg
        self._lat_min = message.latitude_min
        self._long_deg = message.longitude_deg
        self._long_min = message.longitude_min
        self._track_angle = message.bearing_deg
        self._ground_speed = message.speed
        self._time_of_GPS = time.clock()


class nav_status:
    def __init__(self):
        self._navState = None
        self._time_of_status = time.clock()

    def update_nav_status(self, message):
        self._navState = message.nav_state
        self._time_of_IMU = time.clock()
