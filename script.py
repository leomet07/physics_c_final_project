Web VPython 3.2

t=0
dt = 0.001
g = 9.81
coeff_friction = 0.2

xvel_graph = graph(width=350, height=250, xtitle=("Time"), ytitle=("Velocity"), align='left')
xvel_graph_dots =gdots(color=color.red, graph=xvel_graph)

class MyBox():
    def __init__(self, vel, mass):
        self.vel = vel
        self.mass = mass
        self.box = box(pos=vec(0, 0, 0), length=3, height=2, width=1, color=color.cyan) # width is irrelavant
    
    def calculate_acc(self, net_force):
        return net_force * (1.0/self.mass)
    
    def update(self):
        self.box.pos = self.box.pos + self.vel*dt
        
        
        friction_vetor = (-self.mass*g * coeff_friction) * hat(self.vel) # hat of a zero vector is just zero! therefore friction stops when vel=0
        net_force = friction_vetor
        
        acc = self.calculate_acc(net_force)
        
        self.vel = self.vel + acc*dt

mybox = MyBox(vec(10, 0,0), 60)
scene.camera.pos=vector(2, 5, 25) # This tells VPython to view the scene from the position (0,5,10)

while True:
    rate(1/dt)
    
    mybox.update()
    
    xvel_graph_dots.plot(t, mybox.vel.x)
    
    t += dt
