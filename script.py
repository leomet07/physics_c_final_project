Web VPython 3.2

t=0
dt = 0.001
g = 9.81
coeff_friction = 0.2

xvel_graph = graph(width=350, height=250, xtitle=("Time"), ytitle=("Velocity"), align='left')
xvel_graph_dots =gdots(color=color.red, graph=xvel_graph)

wheel_diameter = 622.0 / 1000.0#(mm) /1000
wheel_radius = wheel_diameter * 0.5
wheel_weight = 1 # kg

class MyWheel():
    def __init__(self, a_vel, mass, radius, frame_com_pos, offset_to_com):
        self.a_vel = a_vel
        self.mass = mass
        self.radius = radius
        self.inertia = self.mass * (self.radius**2)
        self.offset_to_com = offset_to_com
        
        self.sphere = sphere(pos=frame_com_pos+offset_to_com, radius=self.radius, color=color.cyan)
    
    def calculate_a_acc(self, net_torque): # in z land bc rotatioin of wheel along x
        return net_torque * (1.0/self.inertia)
    
    def calculate_tangential_velocity(self):
        return vec(self.a_vel.mag * self.radius, 0, 0)# convert z axis rotations to x axis movement
    
    def update(self, frame_com_pos):
        print(frame_com_pos.x)
        self.sphere.pos = frame_com_pos+self.offset_to_com
        
        braking_force_mag = 0 # in newtons, this is an arbitrary braking force, function of hand squeeze
        friction_torque = (-1) * (braking_force_mag) * hat(self.a_vel) # -1 to oppose, hat of a zero vector is just zero! therefore friction stops when a_vel=0
        net_torque = friction_torque
        
        a_acc = self.calculate_a_acc(net_torque)
        
        self.a_vel = self.a_vel + a_acc*dt

class Frame():

    def __init__(self, com_pos, bike_speed_kmh):
        self.visual = sphere(pos=com_pos, radius=0.25, color=color.red)
        bike_speed = bike_speed_kmh * (1000 / 3600)
        self.com_vel = vec(bike_speed, 0, 0)
        
        wheel_rads_per_s = bike_speed / (wheel_radius)
        self.front_wheel = MyWheel(vec(0,0,-wheel_rads_per_s), wheel_weight, wheel_radius, self.visual.pos, vec(0.5, -0.5, 0))
        self.back_wheel = MyWheel(vec(0,0,-wheel_rads_per_s), wheel_weight, wheel_radius, self.visual.pos, vec(-0.5, -0.5, 0))
    
    def update(self):
        self.visual.pos += vec(8.3 * dt, 0,0) # hard coded xvel
        
        # update self, self_com
        self.front_wheel.update(self.visual.pos)
        self.back_wheel.update(self.visual.pos)
        
bike_speed_kmh = 30 # kmh
myframe = Frame(vec(0,0.5,0), bike_speed_kmh)
scene.camera.pos=vector(2, 5, 15) # This tells VPython to view the scene from the position (0,5,10)

while True:
    rate(1/dt)
    
    myframe.update()
    
#    xvel_graph_dots.plot(t, mywheel.calculate_tangential_velocity().x)
    
    t += dt
