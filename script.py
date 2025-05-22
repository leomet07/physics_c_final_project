Web VPython 3.2

t=0
dt = 0.01
g = 9.81
coeff_friction = 0.2

xpos_graph = graph(width=350, height=250, xtitle=("Time"), ytitle=("Xpos"), align='left')
xpos_graph_dots =gdots(color=color.red, graph=xpos_graph)

wheel_diameter = 622.0 / 1000.0#(mm) /1000
wheel_radius = wheel_diameter * 0.5
wheel_weight = 1 # kg

static_friction_constant = 0.65 # for a good tire: https://www.bicyclerollingresistance.com/road-bike-reviews 

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
        self.sphere.pos = frame_com_pos+self.offset_to_com
        
        braking_force_mag = 0 # in newtons, this is an arbitrary braking force, function of hand squeeze
        friction_torque = (-1) * (braking_force_mag) * hat(self.a_vel) # -1 to oppose, hat of a zero vector is just zero! therefore friction stops when a_vel=0
        net_torque = friction_torque
        
        a_acc = self.calculate_a_acc(net_torque)
        
        self.a_vel = self.a_vel + a_acc*dt

class Frame():

    def __init__(self, com_pos, bike_speed_kmh, frame_mass):
        self.visual = sphere(pos=com_pos, radius=0.25, color=color.red)
        bike_speed = bike_speed_kmh * (1000 / 3600)
        self.com_vel = vec(bike_speed, 0, 0)
        self.frame_mass = frame_mass
        
        wheel_rads_per_s = bike_speed / (wheel_radius)
        self.front_wheel = MyWheel(vec(0,0,-wheel_rads_per_s), wheel_weight, wheel_radius, self.visual.pos, vec(0.5, -0.5, 0)) # initial case, match wheel speed to bike translational speed
        self.back_wheel = MyWheel(vec(0,0,-wheel_rads_per_s), wheel_weight, wheel_radius, self.visual.pos, vec(-0.5, -0.5, 0))
    
    def update(self):
        self.visual.pos += self.com_vel * dt # hard coded xvel
        
        total_mass = self.frame_mass + self.front_wheel.mass + self.back_wheel.mass
        
        braking_force = 40 * 6 # roughly 75N (like lfiting a 15lbs weight) from hand, applied across 2cm distance; good rim brakes have mechanichal advantage of 6 
        max_static_friction_force = total_mass * 0.5 * g * static_friction_constant
        print("max_static_friction_force:" , max_static_friction_force)
        if braking_force > max_static_friction_force:
            print("Sliding!")
        static_friction_force = braking_force
        
        if self.com_vel.x <= 0: # if less than 0.18kmh, basically 0
            braking_force = 0
            self.com_vel.x = 0 # keep it at zero forever
        # sources abt mechanichal advatnage: https://www.sheldonbrown.com/cantilever-geometry.html; https://forum.cyclinguk.org/viewtopic.php?t=127211
        sum_of_all_torques_on_front_wheel = -1 * static_friction_force * self.front_wheel.radius # braking force exerted on rim
        change_in_v = (sum_of_all_torques_on_front_wheel * dt) / (total_mass * self.front_wheel.radius)  # times negative one cuz wheel pushes on ground, but ground pushes back in opposite direction
        self.com_vel = self.com_vel + vec(change_in_v, 0, 0)
        if self.com_vel.mag != 0:
            pass
#            print("Position:" , self.visual.pos, "Change in v: ", change_in_v, "Com vel: ", self.com_vel)
        
        # update self, self_com
        self.front_wheel.update(self.visual.pos)
        self.back_wheel.update(self.visual.pos)
        
bike_speed_kmh = 30 # kmh
myframe = Frame(vec(0,0.5,0), bike_speed_kmh, 80)
scene.camera.pos=vector(2, 5, 15) # This tells VPython to view the scene from the position (0,5,10)

while True:
    rate(1/dt)
    
    myframe.update()
    
    xpos_graph_dots.plot(t, myframe.visual.pos.x)
    
    t += dt
