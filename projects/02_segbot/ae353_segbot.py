import numpy as np
import pybullet
import time
import os
import json
import importlib


class Simulator:
    def __init__(
                self,
                display=True,
                seed=None,
                width=640,
                height=480,
                damping=0.,
                tau_max=1.,
                station_velocity=-0.5,
                bumpy=True,
                dt=0.01,
            ):
        
        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = dt

        # Which station model to use
        self.bumpy = bumpy
        if self.bumpy:
            self.station_filename = 'bumpy-station.urdf'
        else:
            self.station_filename = 'station.urdf'

        # Other parameters
        # - Passed
        self.damping = damping
        self.tau_max = tau_max
        self.station_velocity = station_velocity
        # - Hard-coded
        self.wheel_radius = 0.325
        self.wheel_base = 0.7

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            pybullet.connect(pybullet.GUI, options=f'--width={width} --height={height}')
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_GUI, 0,
                lightPosition=[10., 10., 10.],
            )
        else:
            pybullet.connect(pybullet.DIRECT)
        pybullet.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load station
        self.station_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', self.station_filename),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=pybullet.getQuaternionFromEuler([np.pi / 2, 0., 0.]),
            flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                   pybullet.URDF_USE_INERTIA_FROM_FILE  ),
        )

        # Enable velocity control to keep station spinning at fixed rate
        pybullet.setJointMotorControl2(
            self.station_id,
            0,
            pybullet.VELOCITY_CONTROL,
            targetVelocity=self.station_velocity,
            force=1000.,
        )
        
        # Load robot
        self.robot_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'segbot.urdf'),
            basePosition=np.array([0., 0., -20. + (2 * self.wheel_radius) + 0.01]),
            baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
            flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                   pybullet.URDF_USE_INERTIA_FROM_FILE  ))
        
        # Create a dictionary that maps joint names to joint indices and
        # link names to link indices
        self.joint_map = {}
        self.link_map = {}
        for joint_id in range(pybullet.getNumJoints(self.robot_id)):
            joint_name = pybullet.getJointInfo(self.robot_id, joint_id)[1].decode('UTF-8')
            link_name = pybullet.getJointInfo(self.robot_id, joint_id)[12].decode('UTF-8')
            self.joint_map[joint_name] = joint_id
            self.link_map[link_name] = joint_id

        # Create an array with the index of each joint we care about
        self.joint_names = [
            'chassis_to_left_wheel',
            'chassis_to_right_wheel',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to given value
        for id in self.joint_ids:
            pybullet.changeDynamics(self.robot_id, id, jointDamping=damping)

        # Set contact and damping parameters
        for object_id in [self.robot_id, self.station_id]:
            for joint_id in range(-1, pybullet.getNumJoints(object_id)):
                pybullet.changeDynamics(
                    object_id,
                    joint_id,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1,
                    linearDamping=0.,
                    angularDamping=0.,
                )

        # Disable velocity control on each robot joint so we can use torque control
        pybullet.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            pybullet.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints),
        )

        # Set default camera view
        self.camera_backview()
    
    def get_sensor_measurements(self):
        """
        The measurements are:

            lateral error
            heading error
            forward speed
            turning rate
            pitch angle
            pitch rate

        They are computed assuming both wheels roll without slipping.
        """

        # Position of each wheel (center)
        link_states = pybullet.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)

        # Velocity of each wheel
        joint_states = pybullet.getJointStates(self.robot_id, self.joint_ids)
        q = np.zeros([self.num_joints])
        v = np.zeros_like(q)
        for i in range(self.num_joints):
            q[i] = joint_states[i][0]
            v[i] = joint_states[i][1]
        vl = v[0] * self.wheel_radius
        vr = v[1] * self.wheel_radius

        # Lateral error (positive when located too far to left)
        lateral_error = pc[1]

        # Heading error (positive when turned too far to left)
        # - local reference frame
        #  - frame 1 moves with robot around station
        #  - frame 0 is the world frame
        theta = - np.arctan2(pc[0], -pc[2])
        R_1in0 = np.array([[np.cos(theta), 0., np.sin(theta)],
                           [0., 1., 0.],
                           [-np.sin(theta), 0., np.cos(theta)]])
        # - vector from left wheel to right wheel
        a_in0 = pr - pl
        a_in1 = R_1in0.T @ a_in0
        # - heading error
        heading_error = np.arctan2(a_in1[0], -a_in1[1])

        # Forward speed and turning rate
        forward_speed = (vr + vl) / 2.0
        turning_rate = (vr - vl) / np.linalg.norm(pr - pl)

        # Position, orientation, and angular velocity of chassis
        pos, ori = pybullet.getBasePositionAndOrientation(self.robot_id)
        vel = pybullet.getBaseVelocity(self.robot_id)
        R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
        w_in_world = np.reshape(np.array(vel[1]), (3, 1))
        w_in_body = R_body_in_world.T @ w_in_world
        R_body_in_1 = R_1in0.T @ R_body_in_world
        pitch_angle = np.arctan2(
            -R_body_in_1[2, 0],
            np.sqrt(R_body_in_1[2, 1]**2 + R_body_in_1[2, 2]**2)
        )
        
        # Pitch angle and pitch rate
        pitch_rate = w_in_body[1, 0] + self.station_velocity

        return lateral_error, heading_error, forward_speed, turning_rate, pitch_angle, pitch_rate
    
    def set_actuator_commands(
                self,
                desired_right_wheel_torque,
                desired_left_wheel_torque
            ):
        
        if not np.isscalar(desired_right_wheel_torque):
            raise Exception('right_wheel_torque must be a scalar')
        
        if not np.isscalar(desired_left_wheel_torque):
            raise Exception('left_wheel_torque must be a scalar')

        right_wheel_torque = np.clip(desired_right_wheel_torque, -self.tau_max, self.tau_max)
        left_wheel_torque = np.clip(desired_left_wheel_torque, -self.tau_max, self.tau_max)
        self.set_joint_torque(np.array([left_wheel_torque, right_wheel_torque]))
        return right_wheel_torque, left_wheel_torque
    
    def set_joint_torque(self, tau):
        if tau.shape[0] != self.num_joints:
            raise Exception('tau must be same length as number of joints')
        zero_gains = tau.shape[0] * (0.,)
        pybullet.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            pybullet.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains,
        )

    def reset(
            self,
            initial_speed=0.,
            initial_lateral_error=0.,
            initial_heading_error=0.,
            initial_pitch=0.,
            station_velocity=-0.5,
        ):

        # Station
        self.station_velocity = station_velocity
        pybullet.resetJointState(
            self.station_id,
            0,
            0.,
            self.station_velocity,
        )
        pybullet.setJointMotorControl2(
            self.station_id,
            0,
            pybullet.VELOCITY_CONTROL,
            targetVelocity=self.station_velocity,
            force=1000.,
        )

        # Robot
        # - position and orientation
        r = 20. - ((1 + np.cos(initial_pitch)) * self.wheel_radius) - 0.01
        theta = 0.
        pos = [np.sin(theta) * r, initial_lateral_error, np.cos(theta) * -r]
        ori = [0., initial_pitch - theta, initial_heading_error]
        pybullet.resetBasePositionAndOrientation(
            self.robot_id,
            pos,
            pybullet.getQuaternionFromEuler(ori)
        )
        # - joint angles and velocities
        angvel_wheels = initial_speed / self.wheel_radius
        for i, joint_id in enumerate(self.joint_ids):
            pybullet.resetJointState(self.robot_id, joint_id, 0., angvel_wheels)
        # - linear and angular velocity
        pybullet.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[
                initial_speed + ((20. - (2 * self.wheel_radius)) * (self.station_velocity)),
                0.,
                0.,
            ],
            angularVelocity=[
                0.,
                -self.station_velocity,
                0.,
            ],
        )
        # - laps (in units of revolutions)
        self.laps = 0.

        # Update camera and display
        self._update_camera()
        self._update_display()
    
    def run(self, controller, max_time=5.0, data_filename=None, video_filename=None, print_debug=False):
        self.data = {
            't': [],
            'lateral_error': [],
            'heading_error': [],
            'forward_speed': [],
            'turning_rate': [],
            'pitch_angle': [],
            'pitch_rate': [],
            'right_wheel_torque': [],
            'right_wheel_torque_command': [],
            'left_wheel_torque': [],
            'left_wheel_torque_command': [],
            'laps': [],
        }
        self.variables_to_log = getattr(controller, 'variables_to_log', [])
        for key in self.variables_to_log:
            if key in self.data.keys():
                raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
            self.data[key] = []

        # Always start from zero time
        self.t = 0.
        self.time_step = 0
        self.max_time_steps = int(max_time / self.dt)
        self.start_time = time.time()

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = int(1 / self.dt)
            print(f'Creating a video with name {video_filename} and fps {fps}')
            w = imageio.get_writer(video_filename,
                                   format='FFMPEG',
                                   mode='I',
                                   fps=fps)

            # Add first frame to video
            rgba = self.snapshot()
            w.append_data(rgba)

        while True:
            all_done = self.step(controller)

            if video_filename is not None:
                if self.time_step % 100 == 0:
                    print(f' {self.time_step} / {self.max_time_steps}')

                # Add frame to video
                rgba = self.snapshot()
                w.append_data(rgba)

            if all_done:
                break

            if (self.max_time_steps is not None) and (self.time_step == self.max_time_steps):
                break

        if video_filename is not None:
            # Close video
            w.close()

        if data_filename is not None:
            with open(data_filename, 'w') as f:
                json.dump(self.data, f)

        stop_time = time.time()
        stop_time_step = self.time_step

        elapsed_time = stop_time - self.start_time
        elapsed_time_steps = stop_time_step
        if (elapsed_time > 0) and print_debug:
            print(f'Simulated {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')

        # convert lists to numpy arrays
        data = self.data.copy()
        for key in data.keys():
            data[key] = np.array(data[key])

        return data

    def update_laps(self):
        # Largest previous angle
        a_pre = self.laps * (2 * np.pi)

        # Current angle
        link_states = pybullet.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta_robot = np.arctan2(pc[0], -pc[2])
        theta_station = pybullet.getJointState(self.station_id, 0)[0]
        a_cur = (theta_robot - theta_station) % (2 * np.pi)

        # Angle increment (if within threshold, increase laps)
        a_inc = (a_cur - a_pre) % (2 * np.pi)
        if (a_inc > 0) and (a_inc < 0.1):
            self.laps = (a_pre + a_inc) / (2 * np.pi)

    def step(self, controller):
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Update laps
        self.update_laps()

        # Get the sensor measurements
        lateral_error, heading_error, forward_speed, turning_rate, pitch_angle, pitch_rate = self.get_sensor_measurements()

        # Get torque commands (run the controller)
        right_wheel_torque_command, left_wheel_torque_command = controller.run(
            self.t,
            lateral_error,
            heading_error,
            forward_speed,
            turning_rate,
            pitch_angle,
            pitch_rate,
        )

        # Apply the torque commands
        right_wheel_torque, left_wheel_torque = self.set_actuator_commands(
            right_wheel_torque_command,
            left_wheel_torque_command,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['lateral_error'].append(lateral_error)
        self.data['heading_error'].append(heading_error)
        self.data['forward_speed'].append(forward_speed)
        self.data['turning_rate'].append(turning_rate)
        self.data['pitch_angle'].append(pitch_angle)
        self.data['pitch_rate'].append(pitch_rate)
        self.data['right_wheel_torque'].append(right_wheel_torque)
        self.data['right_wheel_torque_command'].append(right_wheel_torque_command)
        self.data['left_wheel_torque'].append(left_wheel_torque)
        self.data['left_wheel_torque_command'].append(left_wheel_torque_command)
        self.data['laps'].append(self.laps)
        for key in self.variables_to_log:
            val = getattr(controller, key, np.nan)
            if not np.isscalar(val):
                val = val.flatten().tolist()
            self.data[key].append(val)

        # Try to stay real-time
        if self.display:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.9 * time_to_wait)
                time_to_wait = t - time.time()

        # Take a simulation step
        pybullet.stepSimulation()

        # Increment time step
        self.time_step += 1

        # Update camera
        self._update_camera()

        return all_done

    def snapshot(self):
        link_states = pybullet.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = 20. - (2 * self.wheel_radius)
        pos = [np.sin(theta) * r, 0., np.cos(theta) * -r]

        if self.view == 'sideview':
            (dist, yaw, pitch) = (3., 0., 0.)
            view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(pos, dist, yaw, -pitch, 0., 2)
        elif self.view == 'wideview':
            (dist, yaw, pitch) = (35., 0., 0.)
            view_matrix = pybullet.computeViewMatrixFromYawPitchRoll([0., 0., 0.], dist, yaw, -pitch, 0., 2)
        elif self.view == 'backview':
            (dist, yaw, pitch) = (3., -np.rad2deg(theta) + 90, 0.)
            view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(pos, dist, yaw, -pitch, 0., 1)
        else:
            raise Exception('invalid camera view')
        
        aspect = self.width / self.height
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=90, aspect=aspect, nearVal=0.01, farVal=100.0)
        im = pybullet.getCameraImage(
            self.width, self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
            shadow=1,
            lightDirection=[10., 10., 10.],
        )
        rgba = im[2]
        return rgba

    def _update_display(self):
        # hack to get GUI to update on MacOS
        time.sleep(0.01)
        keys = pybullet.getKeyboardEvents()

    def _update_backview(self):
        link_states = pybullet.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = 20. - (2 * self.wheel_radius)
        pos = [np.sin(theta) * r, 0., np.cos(theta) * -r]
        pybullet.resetDebugVisualizerCamera(3., -np.rad2deg(theta) + 90, 0, pos)
    
    def _update_sideview(self):
        link_states = pybullet.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = 20. - (2 * self.wheel_radius)
        pos = [np.sin(theta) * r, 0., np.cos(theta) * -r]
        pybullet.resetDebugVisualizerCamera(3., 0, 0, pos)
    
    def _update_wideview(self):
        pybullet.resetDebugVisualizerCamera(35., 0, 0, [0., 0., 0.])
    
    def _update_camera(self):
        if self.display:
            if self.view == 'backview':
                self._update_backview()
            elif self.view == 'sideview':
                self._update_sideview()
            elif self.view == 'wideview':
                self._update_wideview()
            else:
                raise Exception('invalid camera view')

    def camera_backview(self):
        self.view = 'backview'
        if self.display:
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP, 1)
            self._update_backview()
            self._update_display()
    
    def camera_sideview(self):
        self.view = 'sideview'
        if self.display:
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP, 0)
            self._update_sideview()
            self._update_display()
    
    def camera_wideview(self):
        self.view = 'wideview'
        if self.display:
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP, 0)
            self._update_wideview()
            self._update_display()
    
    
