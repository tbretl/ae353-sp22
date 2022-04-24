import numpy as np
import pybullet
import time
import os
from scipy import linalg
import importlib
import pkgutil
import traceback
import contextlib
import io


class Simulator:

    def __init__(
                    self,
                    display=True,
                    seed=None,
                    pos_noise=0.01,
                    yaw_noise=0.001,
                    num_rings=5,
                    ring_separation=5.,
                    width=640,
                    height=480,
                ):

        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = 0.01

        # Whether or not to error on controller print or timeout
        self.error_on_print = True
        self.error_on_timeout = True
        self.max_controller_run_time=5e-3
        self.max_controller_reset_time=1e0
        self.max_controller_init_time=1e0
        self.max_controller_load_time=5e0

        # Create empty list of drones
        self.drones = []
        self.max_num_drones = 40

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            options = f'--width={width} --height={height}'
            pybullet.connect(pybullet.GUI, options=options)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        else:
            pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load plane
        self.plane_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'plane.urdf'),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
            useFixedBase=1,
        )

        # Load rings
        self.num_rings = num_rings
        self.ring_separation = ring_separation
        self.rings = []
        self.add_ring([0., 0., 0.25], [0., -np.pi / 2, 0.], 2.5, 0.5, 'big-ring.urdf')
        for i in range(num_rings):
            x = (i + 1) * ring_separation
            y = self.rng.uniform(low=-5., high=5.)
            z = self.rng.uniform(low=1., high=3.)
            self.add_ring([x, y, z], [0., 0., 0.], 1., 0.25, 'ring.urdf')
        self.add_ring(
            [(num_rings + 1) * ring_separation, 0., 0.25],
            [0., np.pi / 2, 0.],
            2.5, 0.5, 'big-ring.urdf')

        # Set contact parameters
        object_ids = [self.plane_id]
        for ring in self.rings:
            object_ids.append(ring['id'])
        for object_id in object_ids:
            pybullet.changeDynamics(object_id, -1,
                lateralFriction=1.0,
                spinningFriction=0.0,
                rollingFriction=0.0,
                restitution=0.5,
                contactDamping=-1,
                contactStiffness=-1)

        # Camera view
        self.camera_drone_name = None
        self.camera_drone_yaw = None
        self.camera_viewfromstart = True
        self.camera()
        self.update_display()

        # Parameters that govern sensor noise
        self.pos_noise = pos_noise
        self.yaw_noise = yaw_noise

        # Parameters that govern applied force and torque
        #
        #  motor[0]: front (+z spin)
        #  motor[1]: rear (+z spin)
        #  motor[2]: left (-z spin)
        #  motor[3]: right (-z spin)
        #
        self.l = 0.175
        self.kF = 7e-6
        self.kM = 1e-7
        self.min_spin_rate = 100 # <-- rad/s
        self.max_spin_rate = 900 # <-- rad/s
        self.s_min = self.min_spin_rate**2
        self.s_max = self.max_spin_rate**2
        self.M = linalg.inv(np.array([[0., 0., self.kF * self.l, -self.kF * self.l],
                                      [-self.kF * self.l, self.kF * self.l, 0., 0.],
                                      [-self.kM, -self.kM, self.kM, self.kM],
                                      [self.kF, self.kF, self.kF, self.kF]]))

    def set_rules(self, error_on_print=True, error_on_timeout=True):
        self.error_on_print = error_on_print
        self.error_on_timeout = error_on_timeout

    def clear_drones(self):
        for drone in self.drones:
            pybullet.removeBody(drone['id'])
        self.drones = []

    def add_drone(self, Controller, name, image):
        if self.get_drone_by_name(name) is not None:
            raise Exception(f'drone with name "{name}" already exists')
        try:
            # create instance of controller
            controller_start_time = time.time()
            if self.error_on_print:
                with contextlib.redirect_stdout(io.StringIO()) as stdout:
                    controller = Controller()
                stdout_val = stdout.getvalue()
                if stdout_val:
                    raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
            else:
                controller = Controller()
            controller_run_time = time.time() - controller_start_time
            if (controller_run_time > self.max_controller_init_time) and self.error_on_timeout:
                raise Exception(f'Init timeout exceeded: {controller_run_time} > {self.max_controller_init_time}')

            # get color
            color = controller.get_color()
            assert(len(color) == 3)
            color.append(1.)

            # get label
            if image is not None:
                texture_id = pybullet.loadTexture(image)

            # load urdf
            id = pybullet.loadURDF(os.path.join('.', 'urdf', 'drone.urdf'),
                           basePosition=np.array([0., 0., 0.3]),
                           baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                           useFixedBase=0,
                           flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                                  pybullet.URDF_USE_INERTIA_FROM_FILE  ))
            
            # map joint names to joint indices and link names to link indices
            joint_map = {}
            link_map = {}
            link_map['base'] = -1
            for joint_id in range(pybullet.getNumJoints(id)):
                joint_name = pybullet.getJointInfo(id, joint_id)[1].decode('UTF-8')
                link_name = pybullet.getJointInfo(id, joint_id)[12].decode('UTF-8')
                if link_name == 'base':
                    raise Exception('cannot have a non-base link named "base"')
                joint_map[joint_name] = joint_id
                link_map[link_name] = joint_id

            # apply color and label
            pybullet.changeVisualShape(id, link_map['base'], rgbaColor=color)
            if image is None:
                pybullet.changeVisualShape(id, link_map['screen'], rgbaColor=[1., 1., 1., 0.])
            else:
                pybullet.changeVisualShape(id, link_map['screen'], rgbaColor=[1., 1., 1., 0.75], textureUniqueId=texture_id)

            # set contact parameters
            pybullet.changeDynamics(
                id,
                link_map['base'],
                lateralFriction=1.0,
                spinningFriction=0.0,
                rollingFriction=0.0,
                restitution=0.5,
                contactDamping=-1,
                contactStiffness=-1)
            
            # get variables to log
            variables_to_log = getattr(controller, 'variables_to_log', [])

            self.drones.append({
                'id': id,
                'module': None,
                'Controller': Controller,
                'name': name,
                'controller': controller,
                'joint_map': joint_map,
                'link_map': link_map,
                'variables_to_log': variables_to_log,
            })
        except Exception as err:
            print(f'Failed to add {name} because of the following error:')
            print(f'\n==========\n{traceback.format_exc()}==========\n')
    
    
    def load_drones(self, dirname='students', no_max_num_drones=False):
        print(f'Try to import controllers from the directory "./{dirname}":')
        students = importlib.import_module(dirname)
        importlib.reload(students)
        failures = ''
        named_failures = []
        for (_, name, _) in pkgutil.iter_modules([dirname]):
            if (len(self.drones) >= self.max_num_drones) and (not no_max_num_drones):
                raise Exception(f'The simulation already has the maximum number of drones ({self.max_num_drones})')

            print(f' ./{dirname}/{name}.py')
            try:
                # check if drone by this name already exists
                if self.get_drone_by_name(name) is not None:
                    raise Exception(f'drone with name "{name}" already exists')

                # load module
                controller_start_time = time.time()
                module = importlib.import_module(f'.{name}', dirname)
                importlib.reload(module)
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_load_time) and self.error_on_timeout:
                    raise Exception(f'Load timeout exceeded: {controller_run_time} > {self.max_controller_load_time}')

                # create instance of controller
                controller_start_time = time.time()
                if self.error_on_print:
                    with contextlib.redirect_stdout(io.StringIO()) as stdout:
                        controller = module.Controller()
                    stdout_val = stdout.getvalue()
                    if stdout_val:
                        raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
                else:
                    controller = module.Controller()
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_init_time) and self.error_on_timeout:
                    raise Exception(f'Init timeout exceeded: {controller_run_time} > {self.max_controller_init_time}')

                # get color
                color = controller.get_color()
                assert(len(color) == 3)
                color.append(1.)

                # get label
                image = os.path.join('.', dirname, f'{name}.png')
                texture_id = pybullet.loadTexture(image)

                # load urdf
                id = pybullet.loadURDF(os.path.join('.', 'urdf', 'drone.urdf'),
                               basePosition=np.array([0., 0., 0.3]),
                               baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                               useFixedBase=0,
                               flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                                      pybullet.URDF_USE_INERTIA_FROM_FILE  ))

                # map joint names to joint indices and link names to link indices
                joint_map = {}
                link_map = {}
                link_map['base'] = -1
                for joint_id in range(pybullet.getNumJoints(id)):
                    joint_name = pybullet.getJointInfo(id, joint_id)[1].decode('UTF-8')
                    link_name = pybullet.getJointInfo(id, joint_id)[12].decode('UTF-8')
                    if link_name == 'base':
                        raise Exception('cannot have a non-base link named "base"')
                    joint_map[joint_name] = joint_id
                    link_map[link_name] = joint_id

                # apply color and label
                pybullet.changeVisualShape(id, link_map['base'], rgbaColor=color)
                pybullet.changeVisualShape(id, link_map['screen'], rgbaColor=[1., 1., 1., 0.75], textureUniqueId=texture_id)

                # set contact parameters
                pybullet.changeDynamics(id, -1,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1)

                # get variables to log
                variables_to_log = getattr(controller, 'variables_to_log', [])

                self.drones.append({
                    'id': id,
                    'module': module,
                    'Controller': module.Controller,
                    'name': name,
                    'controller': controller,
                    'joint_map': joint_map,
                    'link_map': link_map,
                    'variables_to_log': variables_to_log,
                })
            except Exception as err:
                named_failures.append(name)
                failures += f'\n==========\n{dirname}/{name}.py\n==========\n{traceback.format_exc()}==========\n'
        print(f'\n\nThe following controllers failed to import and were ignored:\n{failures}')
        return named_failures

    def move_rings(self):
        for i in range(self.num_rings):
            x = (i + 1) * self.ring_separation
            y = self.rng.uniform(low=-5., high=5.)
            z = self.rng.uniform(low=1., high=4.)
            self.move_ring([x, y, z], [0., 0., 0.], self.rings[i + 1])

    def move_ring(self, pos, rpy, ring):
        ring['p'] = np.array(pos)
        pybullet.resetBasePositionAndOrientation(ring['id'], pos, pybullet.getQuaternionFromEuler(rpy))

    def add_ring(self, pos, rpy, radius, width, urdf):
        id = pybullet.loadURDF(os.path.join('.', 'urdf', urdf),
                        basePosition=pos,
                        baseOrientation=pybullet.getQuaternionFromEuler(rpy),
                        useFixedBase=1)
        self.rings.append({
            'id': id,
            'p': np.array(pos),
            'R': np.reshape(np.array(pybullet.getMatrixFromQuaternion(pybullet.getQuaternionFromEuler(rpy))), (3, 3)),
            'radius': radius,
            'width': width,
        })

    def is_inside_ring(self, ring, q):
        # Put q in the ring frame
        q = ring['R'].T @ (q - ring['p'])
        # Check if q is too far from y-z plane of ring frame
        if np.abs(q[0]) > (ring['width'] / 2):
            return False
        # Check of q is close enough to x axis of ring frame
        return (q[1]**2 + q[2]**2 <= ring['radius']**2)

    def camera_update(self):
        if self.display:
            if self.camera_drone_name is not None:
                self.camera()

    def camera_update_contest(self, offset=3, yaw=90, pitch=-20):
        if not self.display:
            return

        x = 0.
        for drone in self.drones:
            pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
            if drone['cur_ring'] == len(self.rings):
                xcur = min(pos[0], self.rings[-1]['p'][0])
            else:
                xcur = min(pos[0], self.rings[drone['cur_ring']]['p'][0])
            if xcur > x:
                if (-7.5 < pos[1] < 7.5) and (0. < pos[2] < 5.):
                    x = xcur

        y = 0.
        z = 0.25
        for ring_a, ring_b in zip(self.rings, self.rings[1:]):
            if x <= ring_b['p'][0]:
                m = (x - ring_a['p'][0]) / (ring_b['p'][0] - ring_a['p'][0])
                y = (1 - m) * ring_a['p'][1] + m * ring_b['p'][1]
                z = (1 - m) * ring_a['p'][2] + m * ring_b['p'][2]
                break

        pybullet.resetDebugVisualizerCamera(offset, yaw, pitch, [x, y, z])

    def update_display(self):
        if self.display:
            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = pybullet.getKeyboardEvents()

    def camera(self):
        if self.display:
            if self.camera_drone_name is not None:
                drone = self.get_drone_by_name(self.camera_drone_name)
                if drone is None:
                    raise Exception(f'drone "{drone_name}" does not exist')
                pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
                eul = pybullet.getEulerFromQuaternion(ori)
                pybullet.resetDebugVisualizerCamera(3., (eul[2] * 180 / np.pi) + self.camera_drone_yaw, -15, pos)
            elif self.camera_viewfromstart:
                pybullet.resetDebugVisualizerCamera(5, -90, -30, [0., 0., 0.])
            else:
                pybullet.resetDebugVisualizerCamera(5, 90, -30, self.rings[-1]['p'])

    def camera_startview(self):
        self.camera_viewfromstart = True
        if self.display:
            self.camera_drone_name = None
            self.camera_drone_yaw = None
            self.camera()
            self.update_display()

    def camera_finishview(self):
        self.camera_viewfromstart = False
        if self.display:
            self.camera_drone_name = None
            self.camera_drone_yaw = None
            self.camera()
            self.update_display()

    def camera_droneview(self, drone_name, yaw=270.):
        """
        view from right side: yaw=0
        view from front: yaw=90
        view from left side: yaw=180
        view from back: yaw=270
        """
        if drone_name is None:
            self.camera_drone_name = None
            self.camera_drone_yaw = None
        else:
            if self.get_drone_by_name(drone_name) is None:
                raise Exception(f'drone "{drone_name}" does not exist')
            self.camera_drone_name = drone_name
            self.camera_drone_yaw = yaw
        if self.display:
            self.camera()
            self.update_display()

    def camera_contestview(self):
        if self.display:
            self.camera_update_contest()
            self.update_display()

    def disconnect(self):
        pybullet.disconnect()

    def reset(self):
        # Reset time
        self.max_time_steps = 0
        self.time_step = 0
        self.t = 0.

        # Do nothing else if there are no drones
        if len(self.drones) == 0:
            return

        # Try to find a place for each drone in the start ring
        p = self._get_points(len(self.drones), 0.25, 2.5)
        if p is None:
            raise Exception('Placement failed! Try again.')

        # Set the initial state of each drone
        for drone, point in zip(self.drones, p.tolist()):
            # Position and orientation
            pos = np.array([point[0], point[1], 0.3])
            rpy = 0.01 * self.rng.standard_normal(3)
            ori = pybullet.getQuaternionFromEuler(rpy)
            pybullet.resetBasePositionAndOrientation(drone['id'], pos, ori)
            # Linear and angular velocity
            linvel = 0.01 * self.rng.standard_normal(3)
            angvel = 0.01 * self.rng.standard_normal(3)
            pybullet.resetBaseVelocity(drone['id'],
                                linearVelocity=linvel,
                                angularVelocity=angvel)
            # Actuator commands
            drone['u'] = np.zeros(4)
            # Index of target ring
            drone['cur_ring'] = 1
            # Data
            drone['data'] = {
                't': [],
                'p_x': [],
                'p_y': [],
                'p_z': [],
                'yaw': [],
                'pitch': [],
                'roll': [],
                'v_x': [],
                'v_y': [],
                'v_z': [],
                'w_x': [],
                'w_y': [],
                'w_z': [],
                'p_x_meas': [],
                'p_y_meas': [],
                'p_z_meas': [],
                'yaw_meas': [],
                'p_x_ring': [],
                'p_y_ring': [],
                'p_z_ring': [],
                'is_last_ring': [],
                'tau_x': [],
                'tau_y': [],
                'tau_z': [],
                'f_z': [],
                'tau_x_cmd': [],
                'tau_y_cmd': [],
                'tau_z_cmd': [],
                'f_z_cmd': [],
                'run_time': [],
            }
            # Finish time
            drone['finish_time'] = None
            # Still running
            drone['running'] = True
            # Initialize controller
            try:
                pos_meas = pos + self.pos_noise * self.rng.standard_normal(3)
                yaw_meas = rpy[2] + self.yaw_noise * self.rng.standard_normal()

                controller_start_time = time.time()
                if self.error_on_print:
                    with contextlib.redirect_stdout(io.StringIO()) as stdout:
                        drone['controller'].reset(
                            pos_meas[0],
                            pos_meas[1],
                            pos_meas[2],
                            yaw_meas,
                        )
                    stdout_val = stdout.getvalue()
                    if stdout_val:
                        raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
                else:
                    drone['controller'].reset(
                        pos_meas[0],
                        pos_meas[1],
                        pos_meas[2],
                        yaw_meas,
                    )
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_reset_time) and self.error_on_timeout:
                    raise Exception(f'Reset timeout exceeded: {controller_run_time} > {self.max_controller_reset_time}')
                
                # Try to add user-defined variables to data log
                for key in drone['variables_to_log']:
                    if key in drone['data'].keys():
                        raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
                    drone['data'][key] = []
            except Exception as err:
                print(f'\n==========\nerror on reset of drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

        # Reset camera
        self.camera()
        self.update_display()

    def enforce_motor_limits(self, tau_x_des, tau_y_des, tau_z_des, f_z_des):
        # pack inputs into array
        u = np.array([tau_x_des, tau_y_des, tau_z_des, f_z_des])
        # compute and bound squared spin rates
        s = np.clip(self.M @ u, self.s_min, self.s_max)
        # recompute inputs
        u = linalg.solve(self.M, s)
        return u[0], u[1], u[2], u[3]

    def set_actuator_commands(self, tau_x_des, tau_y_des, tau_z_des, f_z_des, drone):
        tau_x, tau_y, tau_z, f_z = self.enforce_motor_limits(tau_x_des, tau_y_des, tau_z_des, f_z_des)
        drone['u'] = np.array([tau_x, tau_y, tau_z, f_z])
        return tau_x, tau_y, tau_z, f_z

    def get_sensor_measurements(self, drone):
        pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
        pos = np.array(pos)
        rpy = np.array(pybullet.getEulerFromQuaternion(ori))
        pos += self.pos_noise * self.rng.standard_normal(3)
        yaw = rpy[2] + self.yaw_noise * self.rng.standard_normal()
        pos_ring = self.rings[drone['cur_ring']]['p'].copy()
        is_last_ring = ((drone['cur_ring'] + 1) == len(self.rings))
        return pos, yaw, pos_ring, is_last_ring

    def get_state(self, drone):
        pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
        rpy = pybullet.getEulerFromQuaternion(ori)
        vel = pybullet.getBaseVelocity(drone['id'])
        v_world = np.array(vel[0])
        w_world = np.array(vel[1])
        R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
        v_body = R_body_in_world.T @ v_world
        w_body = R_body_in_world.T @ w_world
        return np.array(pos), np.array(rpy), v_body, w_body

    def check_ring(self, drone):
        pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
        pos = np.array(pos)
        if self.is_inside_ring(self.rings[drone['cur_ring']], pos):
            drone['cur_ring'] += 1
        if drone['cur_ring'] == len(self.rings):
            drone['finish_time'] = self.t
            print(f'FINISHED: drone "{drone["name"]}" at time {drone["finish_time"]:.2f}')
            return True
        else:
            return False

    def run(
            self,
            max_time=None,
            video_filename=None,
            contestview=False,
            print_debug=False
        ):

        if max_time is None:
            self.max_time_steps = None
        else:
            self.max_time_steps = int((max_time + self.t) / self.dt)
        self.start_time = time.time() - self.t

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = int(1 / self.dt)
            if print_debug:
                print(f'Creating a video with name {video_filename} and fps {fps}')
            w = imageio.get_writer(video_filename,
                                   format='FFMPEG',
                                   mode='I',
                                   fps=fps)

            # Add first frame to video
            rgba = self.snapshot()
            w.append_data(rgba)

        while True:
            all_done = self.step(contestview=contestview)

            if video_filename is not None:
                if self.time_step % 100 == 0:
                    if print_debug:
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
    

    def get_data(self, drone_name):
        # Try to get drone by name, returning if none exists
        drone = self.get_drone_by_name(drone_name)
        if drone is None:
            drone_names = '\n'.join([d['name'] for d in self.drones])
            msg = f'The simulator has no drone with name "{drone_name}".'
            if len(drone_names) == 0:
                msg += f' The simulator has no drones at all, in fact.'
            else:
                msg += f' The simulator has these drones:'
                msg += f'\n==========\n{drone_names}\n==========\n'
            print(msg)
            return None
        
        # Convert lists to numpy arrays and return the result
        data = drone['data'].copy()
        for key in data.keys():
            data[key] = np.array(data[key])
        return data
    

    def get_result(self, drone_name):
        # Try to get drone by name, returning if none exists
        drone = self.get_drone_by_name(drone_name)
        if drone is None:
            drone_names = '\n'.join([d['name'] for d in self.drones])
            msg = f'The simulator has no drone with name "{drone_name}".'
            if len(drone_names) == 0:
                msg += f' The simulator has no drones at all, in fact.'
            else:
                msg += f' The simulator has these drones:'
                msg += f'\n==========\n{drone_names}\n==========\n'
            print(msg)
            return None
        
        # Get and return the result (did it fail, did it finish, when did it finish)
        if drone['finish_time'] is None:
            if drone['running']:
                failed = False
                finished = False
                finish_time = None
            else:
                failed = True
                finished = False
                finish_time = None
        else:
            failed = False
            finished = True
            finish_time = drone['finish_time']
        return failed, finished, finish_time


    def step(self, contestview=False):
        """
        does one step in the simulation
        """

        # current time
        self.t = self.time_step * self.dt

        # get position of all drones
        all_pos = []
        for drone in self.drones:
            pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
            all_pos.append(pos)
        all_pos = np.array(all_pos)

        all_done = True
        for index, drone in enumerate(self.drones):
            # ignore the drone if it is not still running
            if not drone['running']:
                continue

            # check if the drone has just now finished, and if so ignore it
            if self.check_ring(drone):
                drone['running'] = False
                continue

            # the drone is not finished, so the simulation should continue
            all_done = False

            # get state
            pos, rpy, linvel, angvel = self.get_state(drone)

            # get measurements
            pos_meas, yaw_meas, pos_ring, is_last_ring = self.get_sensor_measurements(drone)

            # get actuator commands
            try:
                controller_start_time = time.time()
                if self.error_on_print:
                    with contextlib.redirect_stdout(io.StringIO()) as stdout:
                        (
                            tau_x_cmd,
                            tau_y_cmd,
                            tau_z_cmd,
                            f_z_cmd,
                        ) = drone['controller'].run(
                            pos_meas[0],
                            pos_meas[1],
                            pos_meas[2],
                            yaw_meas,
                            pos_ring[0],
                            pos_ring[1],
                            pos_ring[2],
                            is_last_ring,
                            np.delete(all_pos, index, axis=0),
                        )
                    stdout_val = stdout.getvalue()
                    if stdout_val:
                        raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
                else:
                    (
                        tau_x_cmd,
                        tau_y_cmd,
                        tau_z_cmd,
                        f_z_cmd,
                    ) = drone['controller'].run(
                        pos_meas[0],
                        pos_meas[1],
                        pos_meas[2],
                        yaw_meas,
                        pos_ring[0],
                        pos_ring[1],
                        pos_ring[2],
                        is_last_ring,
                        np.delete(all_pos, index, axis=0),
                    )
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_run_time) and self.error_on_timeout:
                    raise Exception(f'Run timeout exceeded: {controller_run_time} > {self.max_controller_run_time}')

                (
                    tau_x,
                    tau_y,
                    tau_z,
                    f_z,
                ) = self.set_actuator_commands(
                    tau_x_cmd,
                    tau_y_cmd,
                    tau_z_cmd,
                    f_z_cmd,
                    drone,
                )
            except Exception as err:
                print(f'\n==========\nerror on run of drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

            # apply rotor forces
            pybullet.applyExternalForce(
                drone['id'],
                drone['link_map']['center_of_mass'],
                np.array([0., 0., drone['u'][3]]),
                np.array([0., 0., 0.]),
                pybullet.LINK_FRAME,
            )

            # apply rotor torques
            pybullet.applyExternalTorque(
                drone['id'],
                drone['link_map']['center_of_mass'],
                np.array([drone['u'][0], drone['u'][1], drone['u'][2]]),
                pybullet.LINK_FRAME,
            )

            # log data
            data = drone['data']
            data['t'].append(self.t)
            data['p_x'].append(pos[0])
            data['p_y'].append(pos[1])
            data['p_z'].append(pos[2])
            data['yaw'].append(rpy[2])
            data['pitch'].append(rpy[1])
            data['roll'].append(rpy[0])
            data['v_x'].append(linvel[0])
            data['v_y'].append(linvel[1])
            data['v_z'].append(linvel[2])
            data['w_x'].append(angvel[0])
            data['w_y'].append(angvel[1])
            data['w_z'].append(angvel[2])
            data['p_x_meas'].append(pos_meas[0])
            data['p_y_meas'].append(pos_meas[1])
            data['p_z_meas'].append(pos_meas[2])
            data['yaw_meas'].append(yaw_meas)
            data['p_x_ring'].append(pos_ring[0])
            data['p_y_ring'].append(pos_ring[1])
            data['p_z_ring'].append(pos_ring[2])
            data['is_last_ring'].append(is_last_ring)
            data['tau_x'].append(tau_x)
            data['tau_y'].append(tau_y)
            data['tau_z'].append(tau_z)
            data['f_z'].append(f_z)
            data['tau_x_cmd'].append(tau_x_cmd)
            data['tau_y_cmd'].append(tau_y_cmd)
            data['tau_z_cmd'].append(tau_z_cmd)
            data['f_z_cmd'].append(f_z_cmd)
            data['run_time'].append(controller_run_time)
            try:
                for key in drone['variables_to_log']:
                    val = getattr(drone['controller'], key, np.nan)
                    if not np.isscalar(val):
                        val = val.flatten().tolist()
                    data[key].append(val)
            except Exception as err:
                print(f'\n==========\nerror logging data for drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

        # try to stay real-time
        if self.display:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.9 * time_to_wait)
                time_to_wait = t - time.time()

        # take a simulation step
        pybullet.stepSimulation()

        # increment time step
        self.time_step += 1

        # update camera
        if contestview:
            self.camera_update_contest()
        else:
            self.camera_update()

        return all_done

    def get_drone_by_name(self, name):
        for drone in self.drones:
            if drone['name'] == name:
                return drone
        return None

    def show_results(self):
        finished = []
        still_running = []
        failed = []
        for drone in self.drones:
            if drone['finish_time'] is not None:
                finished.append((drone['name'], drone['finish_time']))
            elif drone['running']:
                still_running.append(drone['name'])
            else:
                failed.append(drone['name'])

        sorted(finished, key=lambda f: f[1])
        print('FINISHED')
        for d in finished:
            print(f' {d[0]:20s} : {d[1]:6.2f}')

        print('\nSTILL RUNNING')
        for d in still_running:
            print(f' {d:20s}')

        print('\nFAILED')
        for d in failed:
            print(f' {d:20s}')

    def snapshot(self):
        # Note: you *must* specify a projectionMatrix when calling getCameraImage,
        # or you will get whatever view is currently shown in the GUI.

        # World view
        if self.camera_viewfromstart:
            p_eye = np.array([-3.5, 0., 2.])
            p_target = np.array(self.rings[0]['p'])
        else:
            p_eye = np.array([-3.5, 0., 2.])
            p_target = np.array(self.rings[-1]['p'])
        v_up = np.array([0., 0., 1.])
        view_matrix = pybullet.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=120, aspect=1.0, nearVal=0.01, farVal=100.0)
        im = pybullet.getCameraImage(480, 480, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=1)
        rgba_world = im[2]

        # Body view (picture-in-picture)
        if self.camera_drone_name is not None:
            drone = self.get_drone_by_name(self.camera_drone_name)
            if drone is None:
                raise Exception(f'drone "{drone_name}" does not exist')
            pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
            o_body_in_world = np.array(pos)
            R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
            p_eye = o_body_in_world + R_body_in_world @ np.array([-1.5, 0., 0.5])
            p_target = o_body_in_world + R_body_in_world @ np.array([0.5, 0., 0.])
            v_up = (R_body_in_world[:, 2]).flatten()
            view_matrix = pybullet.computeViewMatrix(p_eye, p_target, v_up)
            projection_matrix = pybullet.computeProjectionMatrixFOV(fov=60.0, aspect=1.0, nearVal=0.01, farVal=100.0)
            im = pybullet.getCameraImage(128, 128, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=0)
            rgba_body = im[2]
            rgba_world[10:138, 10:138, :] = rgba_body

        return rgba_world

    def _get_rep_grad(self, p, i, params):
        gradfrep = np.zeros(2)
        for j in range(p.shape[0]):
            if j != i:
                v = p[i] - p[j]
                vnorm = np.linalg.norm(v)
                d = vnorm
                dgrad = v / vnorm
                if (d <= params['brep']):
                    gradfrep += params['krep'] * ((1 / params['brep']) - (1 / d)) * (1 / (d ** 2)) * dgrad
        v = p[i]
        vnorm = np.linalg.norm(v)
        d = params['radius'] - vnorm
        dgrad = - v / vnorm
        if (d <= params['brep']):
            gradfrep += params['krep'] * ((1 / params['brep']) - (1 / d)) * (1 / (d ** 2)) * dgrad
        d = np.linalg.norm(gradfrep)
        if (d >= params['max_step']):
            gradfrep *= (params['max_step'] / d)
        return gradfrep

    def _get_step(self, p, params):
        dp = []
        for i in range(p.shape[0]):
            dp.append(- params['kdes'] * self._get_rep_grad(p, i, params))
        return np.array(dp)

    def _get_dmin_for_point(self, p, j, params):
        dmin = params['radius'] - np.linalg.norm(p[j])
        for i in range(p.shape[0]):
            if i != j:
                d = np.linalg.norm(p[i] - p[j])
                if d < dmin:
                    dmin = d
        return dmin

    def _get_dmin(self, p, params):
        dmin = np.inf
        for i in range(p.shape[0]):
            d = self._get_dmin_for_point(p, i, params)
            if d < dmin:
                dmin = d
        return dmin

    def _get_points(self, num_points, inner_radius, outer_radius):
        # sample points in circle
        pr = self.rng.uniform(low=0., high=outer_radius, size=(num_points,))
        ph = self.rng.uniform(low=0., high=2*np.pi, size=(num_points,))
        p = (pr * np.array([np.cos(ph), np.sin(ph)])).T
        # do 25 steps of gradient descent to spread out the points
        params = {
            'krep': 1.,
            'brep': 4 * inner_radius,
            'katt': 1.,
            'batt': 1.,
            'kdes': 5e-1,
            'radius': outer_radius,
            'max_step': 0.1,
        }
        for i in range(50):
            p += self._get_step(p, params)
        if self._get_dmin(p, params) > 2 * inner_radius:
            return p
        else:
            return None
