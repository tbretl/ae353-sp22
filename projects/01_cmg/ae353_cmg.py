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
                roll=(np.pi / 2),
                rotor_velocity=1000.,
                load_mass=1.,
                damping=0.,
                tau_max=5.,
                dt=0.01,
            ):
        
        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = 0.01

        # Other parameters
        self.roll = roll
        self.load_mass = load_mass
        self.damping = damping
        self.tau_max = tau_max

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
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Define the camera view
        pybullet.resetDebugVisualizerCamera(2.0, 50, -35, (0., 0., 0.))

        # Load plane
        self.plane_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'plane.urdf'),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=pybullet.getQuaternionFromEuler([0., self.roll, 0.]),
            useFixedBase=1,
        )
        
        # Load cmg
        self.cmg_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'cmg.urdf'),
            flags=(
                    pybullet.URDF_USE_IMPLICIT_CYLINDER |
                    pybullet.URDF_USE_INERTIA_FROM_FILE
            ),
            basePosition=np.array([1.1 * np.sin(self.roll), 0., 1.1 * np.cos(self.roll)]),
            baseOrientation=pybullet.getQuaternionFromEuler([np.pi / 2, self.roll, 0.]),
        )

        # Create a dictionary that maps joint names to joint indices and
        # link names to link indices
        self.joint_map = {}
        self.link_map = {}
        for joint_id in range(pybullet.getNumJoints(self.cmg_id)):
            joint_name = pybullet.getJointInfo(self.cmg_id, joint_id)[1].decode('UTF-8')
            link_name = pybullet.getJointInfo(self.cmg_id, joint_id)[12].decode('UTF-8')
            self.joint_map[joint_name] = joint_id
            self.link_map[link_name] = joint_id

        # Create an array with the index of each joint we care about
        self.joint_names = [
            'world_to_outer',
            'outer_to_inner',
            'inner_to_wheel',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to given value
        for id in self.joint_ids:
            pybullet.changeDynamics(self.cmg_id, id, jointDamping=damping)
        
        self.set_rotor_velocity(rotor_velocity)

        # Disable velocity control on each joint so we can use torque control
        pybullet.setJointMotorControlArray(
            self.cmg_id,
            self.joint_ids,
            pybullet.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints)
        )

        # Eliminate linear and angular damping (a poor model of drag)
        pybullet.changeDynamics(self.cmg_id, -1, linearDamping=0., angularDamping=0.)
        for joint_id in range(pybullet.getNumJoints(self.cmg_id)):
            pybullet.changeDynamics(self.cmg_id, joint_id, linearDamping=0., angularDamping=0.)
        
        # Change load mass
        pybullet.changeDynamics(self.cmg_id, self.link_map['mass'], mass=load_mass)
        
        # Camera view
        self.camera_sideview()
    
    def set_rotor_velocity(self, rotor_velocity):
        # Increase maximum joint velocity of rotor joint - this is capped, by
        # default, to a value of 100 for numerical stability. This is very bad
        # for the CMG, because it results in a very large constraint torque
        # being applied (without you knowing it) to the rotor, which totally
        # breaks everything. If you don't do this, nothing will work! We do
        # not have to worry about "numerical stability" in this case, because
        # (for example) there isn't any chance of collision between objects.
        if np.abs(rotor_velocity) > 100.:
            pybullet.changeDynamics(
                self.cmg_id,
                self.joint_map['inner_to_wheel'],
                maxJointVelocity=(10. * np.abs(rotor_velocity)),
            )
        self.rotor_velocity = rotor_velocity

    def get_sensor_measurements(self):
        joint_states = pybullet.getJointStates(self.cmg_id, self.joint_ids)
        platform_angle = joint_states[0][0]
        platform_velocity = joint_states[0][1]
        gimbal_angle = joint_states[1][0]
        gimbal_velocity = joint_states[1][1]
        rotor_velocity = joint_states[2][1]
        return platform_angle, platform_velocity, gimbal_angle, gimbal_velocity, rotor_velocity
    
    def set_actuator_commands(self, desired_gimbal_torque, desired_rotor_torque):
        assert(np.isscalar(desired_gimbal_torque))
        assert(np.isscalar(desired_rotor_torque))
        gimbal_torque = np.clip(desired_gimbal_torque, -self.tau_max, self.tau_max)
        rotor_torque = np.clip(desired_rotor_torque, -self.tau_max, self.tau_max)
        self.set_joint_torque(np.array([0., gimbal_torque, rotor_torque]))
        return gimbal_torque, rotor_torque
    
    def set_joint_torque(self, tau):
        assert(tau.shape[0] == self.num_joints)
        zero_gains = tau.shape[0] * (0.,)
        pybullet.setJointMotorControlArray(
            self.cmg_id,
            self.joint_ids,
            pybullet.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains
        )

    def reset(
            self,
            platform_angle=0.,
            platform_velocity=0.,
            gimbal_angle=0.,
            gimbal_velocity=0.,
            rotor_velocity=None,
        ):
        
        pybullet.resetJointState(
            self.cmg_id,
            self.joint_map['world_to_outer'],
            platform_angle,
            platform_velocity,
        )

        pybullet.resetJointState(
            self.cmg_id,
            self.joint_map['outer_to_inner'],
            gimbal_angle,
            gimbal_velocity,
        )

        rotor_angle = 0.
        if rotor_velocity is None:
            rotor_velocity = self.rotor_velocity
        self.set_rotor_velocity(rotor_velocity)
        pybullet.resetJointState(
            self.cmg_id,
            self.joint_map['inner_to_wheel'],
            rotor_angle,
            rotor_velocity,
        )

        self.camera()
    
    def run(self, controller, max_time=5.0, data_filename=None, video_filename=None, print_debug=False):
        self.data = {
            't': [],
            'platform_angle': [],
            'platform_velocity': [],
            'gimbal_angle': [],
            'gimbal_velocity': [],
            'gimbal_torque': [],
            'gimbal_torque_command': [],
            'rotor_velocity': [],
            'rotor_torque': [],
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

    def step(self, controller):
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Get the sensor measurements
        platform_angle, platform_velocity, gimbal_angle, gimbal_velocity, rotor_velocity = self.get_sensor_measurements()

        # Get the gimbal torque command (run the controller)
        gimbal_torque_command = controller.run(
            self.t,
            platform_angle,
            platform_velocity,
            gimbal_angle,
            gimbal_velocity,
            rotor_velocity,
        )

        # Get the rotor torque command (internal)
        rotor_torque_command = -1. * (rotor_velocity - self.rotor_velocity)

        # Apply the torque commands
        gimbal_torque, rotor_torque = self.set_actuator_commands(
            gimbal_torque_command,
            rotor_torque_command,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['platform_angle'].append(platform_angle)
        self.data['platform_velocity'].append(platform_velocity)
        self.data['gimbal_angle'].append(gimbal_angle)
        self.data['gimbal_velocity'].append(gimbal_velocity)
        self.data['gimbal_torque'].append(gimbal_torque)
        self.data['gimbal_torque_command'].append(gimbal_torque_command)
        self.data['rotor_velocity'].append(rotor_velocity)
        self.data['rotor_torque'].append(rotor_torque)
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

        return all_done

    def snapshot(self):
        pos = self.camera_target
        yaw = -90 + self.camera_yaw
        aspect = self.width / self.height
        view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(pos, self.camera_distance, yaw, -self.camera_pitch, 0., 2)
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

    def camera(self):
        if self.display:
            pybullet.resetDebugVisualizerCamera(self.camera_distance, -90 + self.camera_yaw, -self.camera_pitch, self.camera_target)

            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = pybullet.getKeyboardEvents()

    def camera_topview(self):
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_distance = 5.
        self.camera_pitch = np.clip(90. - np.rad2deg(self.roll), -89.9, 89.9)
        self.camera_yaw = 180.
        self.camera()

    def camera_sideview(self):
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_distance = 4.
        self.camera_pitch = 15.
        self.camera_yaw = 150.
        self.camera()