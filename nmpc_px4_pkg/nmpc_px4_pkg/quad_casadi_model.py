from casadi import SX, vertcat, cos, sin

class Quadrotor:
    def __init__(self, sim: bool):
        self.sim = sim
        self.g = 9.806 #gravity
        self.m = 1.5 if sim else 2.0 #hardware mass could be up to 2.19kg

    def dynamics(self):
        #states
        px = SX.sym('px')
        py = SX.sym('py')
        pz = SX.sym('pz')
        vx = SX.sym('vx')
        vy = SX.sym('vy')
        vz = SX.sym('vz')
        roll = SX.sym('roll')
        pitch = SX.sym('pitch')
        yaw = SX.sym('yaw')

        #state vector
        x = vertcat(px, py, pz, vx, vy, vz, roll, pitch, yaw)
        
        #control inputs
        thrust = SX.sym('thrust')
        rolldot = SX.sym('rolldot')
        pitchdot = SX.sym('pitchdot')
        yawdot = SX.sym('yawdot')

        #control vector
        u = vertcat(thrust, rolldot, pitchdot, yawdot)

        # define trig functions
        sr = sin(roll)
        sy = sin(yaw)
        sp = sin(pitch)
        cr = cos(roll)
        cp = cos(pitch)
        cy = cos(yaw)

        #define dynamics
        pxdot = vx
        pydot = vy
        pzdot = vz
        vxdot = -(thrust/self.m) * (sr*sy + cr*cy*sp)
        vydot = -(thrust/self.m) * (cr*sy*sp - cy*sr)
        vzdot = self.g - (thrust/self.m) * (cr*cp)
        rolldot = rolldot
        pitchdot = pitchdot
        yawdot = yawdot

        f_expl = vertcat(pxdot, pydot, pzdot, vxdot, vydot, vzdot, rolldot, pitchdot, yawdot)

        return (f_expl, x, u)