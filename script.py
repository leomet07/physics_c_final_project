Web VPython 3.2

t=0
dt = 0.001
g = 9.81
coeff_friction = 0.2

xvel_graph = graph(width=350, height=250, xtitle=("Time"), ytitle=("Velocity"), align='left')
xvel_graph_dots =gdots(color=color.red, graph=xvel_graph)

class MyWheel():
    def __init__(self, a_vel, mass, radius):
        self.a_vel = a_vel
        self.mass = mass
        self.radius = radius
        self.inertia = self.mass * (self.radius**2)
        
        self.sphere = sphere(pos=vec(0, 0, 0), radius=self.radius, color=color.cyan)
    
    def calculate_a_acc(self, net_torque): # in z land bc rotatioin of wheel along x
        return net_torque * (1.0/self.inertia)
    
    def calculate_tangential_velocity(self):
        return vec(self.a_vel.mag * self.radius, 0, 0)# convert z axis rotations to x axis movement
    
    def update(self):
        self.sphere.pos = self.sphere.pos + self.calculate_tangential_velocity()*dt
        
        braking_force_mag = 20 # in newtons, this is an arbitrary braking force, function of hand squeeze
        friction_torque = (-1) * (braking_force_mag) * hat(self.a_vel) # -1 to oppose, hat of a zero vector is just zero! therefore friction stops when a_vel=0
        net_torque = friction_torque
        
        a_acc = self.calculate_a_acc(net_torque)
        
        self.a_vel = self.a_vel + a_acc*dt

wheel_diameter = 622.0 / 1000.0#(mm) /1000
wheel_radius = wheel_diameter * 0.5
wheel_weight = 1 # kg

bike_speed_kmh = 25 # km/h
bike_speed = bike_speed_kmh * (1000 / 3600)
bike_rpm = bike_speed / (2.0 * pi * wheel_radius)

mywheel = MyWheel(vec(0, 0, -4), wheel_weight, wheel_radius)
scene.camera.pos=vector(2, 5, 15) # This tells VPython to view the scene from the position (0,5,10)

while True:
    rate(1/dt)
    
    mywheel.update()
    
    xvel_graph_dots.plot(t, mywheel.calculate_tangential_velocity().x)
    
    t += dt
