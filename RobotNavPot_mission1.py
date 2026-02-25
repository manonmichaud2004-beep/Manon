# -*- coding: utf-8 -*-
"""
Way Point navigtion

(c) S. Bertrand
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# robot
x0 = -20.0
y0 = -20.0
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=1, random=True)


# position control loop: gain and timer
kpPos = 0.8
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 2.5
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)



# list of way points list of [x coord, y coord]
WPlist = [ [x0,y0] ]
#threshold for change to next WP
epsilonWP = 0.2
# init WPManager
WPManager = rob.WPManager(WPlist, epsilonWP)


# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 200.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

firstIter = True



# Mission 1

# paramètres
R_MISSION = 1.0 
V_VITESSE = 1.0  


W_CERCLE = V_VITESSE / R_MISSION
TEMPS_UN_TOUR = (2 * math.pi) / W_CERCLE

def piloter_robot(v, w):
    robot.setV(v)
    robot.setOmega(w)
    robot.integrateMotion(dt)
    val = pot.value([robot.x, robot.y])
    simu.addData(robot, WPManager, v, robot.theta, w, val)
    return val


for cycle in range(2):
    p_min, p_max = float('inf'), float('-inf')
    pos_min, pos_max = [0,0], [0,0]

    
    for t in np.arange(0, TEMPS_UN_TOUR, dt):
        val = piloter_robot(V_VITESSE, W_CERCLE)
        if val > p_max: p_max, pos_max = val, [robot.x, robot.y]
        if val < p_min: p_min, pos_min = val, [robot.x, robot.y]

    # retour au point min
    while math.sqrt((robot.x - pos_min[0])**2 + (robot.y - pos_min[1])**2) > 0.2:
        piloter_robot(V_VITESSE, W_CERCLE)

    # orientation vers le max
    angle_vers_pic = math.atan2(pos_max[1] - robot.y, pos_max[0] - robot.x)
    while True:
        erreur = (angle_vers_pic - robot.theta + math.pi) % (2 * math.pi) - math.pi
        if abs(erreur) < 0.05: break
        piloter_robot(0.0, 2.0 * erreur)

    # avance vers le Max jusqu'à redescente
    p_prec = 0
    while True:
        val = piloter_robot(V_VITESSE, 0.0)
        if val < p_prec - 0.001: 
            break
        p_prec = val
    
    print(f"Cycle {cycle + 1} terminé avec un niveau atteint de : {p_prec:.2f}")
    
    
# Mission 2

target_level = 200.0
vitesse = 1.2
depart_trace = None
deja_loin = False
p_precedente = pot.value([robot.x, robot.y])

while simu.currentIndex < len(simu.t):
    p_actuelle = pot.value([robot.x, robot.y])
    

    # puisque le robot bouge la différence de pollution nous donne la pente
    delta_p = p_actuelle - p_precedente
    
    # direction
    if p_actuelle > target_level + 1.0:
        # On est trop haut : on doit descendre
        # Si delta_p est positif on s'approche du pic -> il faut faire demi-tour
        # Si delta_p est négatif on s'éloigne du pic -> on est dans le bon sens
        omega = 2.0 if delta_p > 0 else 0.0
    else:
        # on est au bon niveau on commence le suivi
        if depart_trace is None:
            depart_trace = [robot.x, robot.y]
        

        # L'erreur ajuste le rayon de braquage
        erreur = p_actuelle - target_level
        omega = erreur 

    # application du mouvement
    robot.setV(vitesse)
    robot.setOmega(omega)
    robot.integrateMotion(dt)
    simu.addData(robot, WPManager, vitesse, robot.theta, omega, p_actuelle)
    
    # Mise à jour de la mémoire
    p_precedente = p_actuelle

    # 4. Condition d'arrêt
    if depart_trace is not None:
        dist = math.sqrt((robot.x - depart_trace[0])**2 + (robot.y - depart_trace[1])**2)
        if dist > 5.0:
            deja_loin = True
        if deja_loin and dist < 1.2:
            print("Mission 2 terminée")
            break

robot.setV(0.0)
robot.setOmega(0.0)



# end of loop on simulation time


# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

simu.plotXYTheta(2)
#simu.plotVOmega(3)

simu.plotPotential(4)



simu.plotPotential3D(5)


# show plots
#plt.show()





# # Animation *********************************
# fig = plt.figure()
# ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
# ax.grid()
# ax.set_xlabel('x (m)')
# ax.set_ylabel('y (m)')

# robotBody, = ax.plot([], [], 'o-', lw=2)
# robotDirection, = ax.plot([], [], '-', lw=1, color='k')
# wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
# time_template = 'time = %.1fs'
# time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
# potential_template = 'potential = %.1f'
# potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
# WPArea, = ax.plot([], [], ':', lw=1, color='b')

# thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
# xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
# yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

# def initAnimation():
#     robotDirection.set_data([], [])
#     robotBody.set_data([], [])
#     wayPoint.set_data([], [])
#     WPArea.set_data([], [])
#     robotBody.set_color('r')
#     robotBody.set_markersize(20)    
#     time_text.set_text('')
#     potential_text.set_text('')
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  
    
# def animate(i):  
#     robotBody.set_data(simu.x[i], simu.y[i])          
#     wayPoint.set_data(simu.xr[i], simu.yr[i])
#     WPArea.set_data(simu.xr[i]+xWPArea.transpose(), simu.yr[i]+yWPArea.transpose())    
#     thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
#     thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
#     robotDirection.set_data(thisx, thisy)
#     time_text.set_text(time_template%(i*simu.dt))
#     potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea

# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
#     interval=4, blit=True, init_func=initAnimation, repeat=False)
# #interval=25

# #ani.save('robot.mp4', fps=15)

