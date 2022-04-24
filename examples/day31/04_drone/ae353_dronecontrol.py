import numpy as np

class Controller:
    def __init__(self):
        
        # Define all variables (FIXME)
        #
        # self.A = ...
        # self.B = ...
        # ...
        # ...
        
        self.variables_to_log = ['xhat', 'xdes', 'ring']

    def get_color(self):
        return [1., 0., 0.]

    def reset(
            self,
            p_x_meas, p_y_meas, p_z_meas, # <-- measured position of drone (meters)
            yaw_meas,                     # <-- measured yaw angle of drone (radians)
        ):
        
        # Choose initial state estimate (FIXME)
        #
        self.xhat = np.zeros(12)
        

    def run(
            self,
            p_x_meas, p_y_meas, p_z_meas, # <-- measured position of drone (meters)
            yaw_meas,                     # <-- measured yaw angle of drone (radians)
            p_x_ring, p_y_ring, p_z_ring, # <-- center position of next ring (meters)
            is_last_ring,                 # <-- True if next ring is the last ring, False otherwise
            pos_others,                   # <-- 2d array of size n x 3, where n is the number
                                          #     of all *other* drones - the ith row in this array
                                          #     has the coordinates [x_i, y_i, z_i], in meters, of
                                          #     the ith other drone
        ):
        
        # Store ring position for later analysis
        self.ring = np.array([p_x_ring, p_y_ring, p_z_ring])
        
        # Get xdes and udes
        self.xdes, self.udes = self.get_xdes_and_udes(self.xhat, p_x_ring, p_y_ring, p_z_ring)
        
        # Apply controller and observer... (FIXME)
        #
        # u = ...
        # y = ...
        # self.xhat += ...
        
        tau_x = 0.
        tau_y = 0.
        tau_z = 0.
        f_z = 0.
        
        return tau_x, tau_y, tau_z, f_z
    
    def get_xdes_and_udes(self, xhat, p_x_ring, p_y_ring, p_z_ring):
        xdes = np.zeros(12)
        udes = np.zeros(4)
        return xdes, udes