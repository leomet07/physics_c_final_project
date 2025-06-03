Web VPython 3.2

t=0
dt = 0.02
g = 9.81

xpos_graph = graph(width=350, height=250, xtitle=("Time"), ytitle=("COM x pos"), align='left')
xpos_graph_dots =gdots(color=color.red, graph=xpos_graph)
xvel_graph = graph(width=350, height=250, xtitle=("Time"), ytitle=("COM x vel"), align='left')
xvel_graph_dots =gdots(color=color.red, graph=xvel_graph)

wheel_diameter = 622.0 / 1000.0#(mm) /1000
wheel_radius = wheel_diameter * 0.5
wheel_weight = 1 # kg


static_friction_constant = 0.65 # for a good tire: https://www.bicyclerollingresistance.com/road-bike-reviews 
kinetic_friction_constant = 0.25 # for a good tire: https://www.bicyclerollingresistance.com/road-bike-reviews 

def sf_bind(evt):
    global static_friction_constant
    static_friction_constant = evt.value
    sf_text.text = f"Static friction constant: {static_friction_constant}"

sf_slider = slider(bind=sf_bind , max=2, min=0, step=0.05, value=static_friction_constant, id='sf_slider', align="left")
sf_text = wtext(text=f"Static friction constant: {static_friction_constant}")
scene.append_to_caption('\n')
def kf_bind(evt):
    global kinetic_friction_constant
    kinetic_friction_constant = evt.value
    kf_text.text = f"Kinetic friction constant: {kinetic_friction_constant}"

kf_slider = slider(bind=kf_bind , max=2, min=0, step=0.05, value=kinetic_friction_constant, id='kf_slider', align="left")
kf_text = wtext(text=f"Kinetic friction constant: {kinetic_friction_constant}")
scene.append_to_caption('\n')
class MyWheel():
    def __init__(self, a_vel, mass, radius, frame_com_pos, offset_to_com):
        self.a_vel = a_vel
        self.mass = mass
        self.radius = radius
        self.offset_to_com = offset_to_com
        
        self.sphere = sphere(pos=frame_com_pos+offset_to_com, radius=self.radius, color=color.cyan)

    def update(self, frame_com_pos):
        self.sphere.pos = frame_com_pos+self.offset_to_com


class Frame():

    def __init__(self, com_pos, bike_speed_kmh, frame_mass, frame_length, frame_height):
        self.visual = sphere(pos=com_pos, radius=0.25, color=color.red)
        bike_speed = bike_speed_kmh * (1000 / 3600)
        self.com_vel = vec(bike_speed, 0, 0)
        self.frame_mass = frame_mass
        
        wheel_rads_per_s = bike_speed / (wheel_radius)
        self.front_wheel = MyWheel(vec(0,0,-wheel_rads_per_s), wheel_weight, wheel_radius, self.visual.pos, vec(frame_length / 2, -frame_height, 0)) # initial case, match wheel speed to bike translational speed
        self.back_wheel = MyWheel(vec(0,0,-wheel_rads_per_s), wheel_weight, wheel_radius, self.visual.pos, vec(-frame_length / 2, -frame_height, 0))
        self.frame_length = frame_length
        self.frame_height = frame_height
        self.total_mass = self.frame_mass + self.front_wheel.mass + self.back_wheel.mass

        
    def wheelNormalForce(self, frontOrBack):
#        wheel
        if frontOrBack == "front":
            wheel = self.front_wheel
            pos_or_neg = 1
        elif frontOrBack == "back":
            wheel = self.back_wheel
            pos_or_neg = -1
        L = self.frame_length
        bcond = L - abs(wheel.offset_to_com.x)
        h = self.frame_length
        return vec(0,self.total_mass*g*((bcond/L)+((static_friction_constant*h*pos_or_neg)/L)),0)
    
    def update(self):
        self.visual.pos += self.com_vel * dt

        Nf = self.wheelNormalForce("front")
        Nb = self.wheelNormalForce("back")
   
        braking_force = 100 * 6 * max(0.5, t) # roughly 75N (like lfiting a 15lbs weight) from hand, applied across 2cm distance; good rim brakes have mechanichal advantage of 6 
        
        max_static_friction_force_front = (Nf * static_friction_constant).mag
        kinetic_friction_force_front = (Nf * kinetic_friction_constant).mag
         
        if self.com_vel.x <= 0: # if not moving (less than 0.18kmh)
            applied_friction_force_on_front_wheel = 0
            self.com_vel.x = 0 # keep it at zero forever
            self.front_wheel.sphere.color = color.cyan
        else:
            if braking_force < max_static_friction_force_front:
                applied_friction_force_on_front_wheel = braking_force # this is supplied by static friction
            else:
                applied_friction_force_on_front_wheel = kinetic_friction_force_front
#                print(f"Sliding at t={t}")
                self.front_wheel.sphere.color = color.yellow
                checkToFlip = True # at this instant, you HAVE locked ur front wheel
        
        # compute sum of all forces on system [NOTE: NO FRICTION FORCE MATCHING BRAKING FORCE ON BACK WHEEL)
        sum_of_all_forces_on_system = Nf + Nb + vec(0,-self.total_mass*g,0) + vec(-applied_friction_force_on_front_wheel,0,0)
        a = sum_of_all_forces_on_system / self.total_mass
        change_in_v = a * dt
        self.com_vel = self.com_vel + change_in_v        
        
        # this is sum of all torques abt pivot at right wheel. should not always be applied...
        # Nf: r x F = 0. Nb: does apply torque. applied force also matters...?
        horiz_to_front_contact_patch = (self.front_wheel.sphere.pos - vec(0, self.front_wheel.radius, 0))
        sum_of_all_torques_on_system = cross(self.back_wheel.sphere.pos - horiz_to_front_contact_patch, Nb) + cross(self.visual.pos - (self.front_wheel.sphere.pos - vec(0, self.front_wheel.radius, 0)),vec(0,-self.total_mass*g,0))
#        print(sum_of_all_torques_on_system)
        if checkToFlip:
            r = self.visual.pos - horiz_to_front_contact_patch
            ratio = abs(r.y / r.x)
            print(ratio, (1/static_friction_constant))
            if ratio > (1/static_friction_constant):
                print("FLIP TIME")
                
            
        
        # update self, self_com
        self.front_wheel.update(self.visual.pos)
        self.back_wheel.update(self.visual.pos)
        
bike_speed_kmh = 30 # kmh
myframe = Frame(vec(0,0.5,0), bike_speed_kmh, 80, 1, 0.5)
scene.camera.pos=vector(8, 5, 12)

# draw grid
grid_line_x_spacing=5
for x in range(0, 50, grid_line_x_spacing):
    if x == 0:
        linecolor = color.blue
    else:
        linecolor = color.white
    curve(pos=[vec(x, -1, -0.001), vec(x, 1, -0.001)], color=linecolor)
    label(pos=vec(x, -1, 0), text=f"{x}m", xoffset=0, yoffset=0, space=30, height=16, border=4, font='sans')
# draw floor
curve(pos=[vec(-100, 0, 0), vec(200, 0, 0)], color=color.white)

while True:
    rate(1/dt)
    
    myframe.update()

    xpos_graph_dots.plot(t, myframe.visual.pos.x)
    xvel_graph_dots.plot(t, myframe.com_vel.x)
    
    t += dt
