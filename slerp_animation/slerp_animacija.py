from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
import math
import numpy as np
from zadatak1 import *

name = 'slerp animation'

center1 = np.array([3.6, 4.5, 5.9])
center2 = np.array([0, 0, 5])

phi1 = -0.071655808
theta1 = -0.753012733
psi1 = 1.170186361

phi2 = 0.001
theta2 = 0.001
psi2 = 0.001

TIMER_ID = 0
TIMER_INTERVAL = 150

animation_ongoing = 0
t = 0.0
tm = 1.0

def slerp(q1, q2):
    
    global t
    global tm
    
    cos0 = np.dot(q1, q2)
    
    if(cos0 < 0):
        q1 = -q1
        cos0 = -cos0
        
    if(cos0 > 0.95):
        return q1
    
    phi0 = math.acos(cos0)
    
    qs = (math.sin(phi0*(1-t/tm)))/(math.sin(phi0)) * q1 + (math.sin(phi0*t/tm))/(math.sin(phi0)) * q2
    return qs


def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(700,700)
    glutCreateWindow(name)

    glClearColor(0.,0.,0.,1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    lightZeroPosition = [10.,4.,10.,1.]
    lightZeroColor = [0.8,1.0,0.8,1.0] 
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
    glEnable(GL_LIGHT0)
    
    
    glMatrixMode(GL_PROJECTION)
    gluPerspective(40.,1.,1.,40.)
    glMatrixMode(GL_MODELVIEW)
    gluLookAt(5,6,30,
              0,0,0,
              0,1,0)
    
    glutKeyboardFunc(on_keyboard)
    glutDisplayFunc(display)
    #glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID)
    
    glutMainLoop()
    return


def display():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    
    draw_coordsys()
   
    
    A1 = Euler2A(phi1, theta1, psi1)
    p1, angle1 = AxisAngle(A1)
    q1 = np.array(AxisAngle2Q(p1, angle1))
    
    A2 = Euler2A(phi2, theta2, psi2)
    p2, angle2 = AxisAngle(A2)
    q2 = np.array(AxisAngle2Q(p2, angle2))
    
    q = slerp(q1, q2)
    
    p, angle = Q2AxisAngle(q)
    A = Rodrigez(p, angle)
    angles = A2Euler(A)
    
    center = (1-t)*center1 + t*center2
        
    glPushMatrix()
    glTranslatef(center[0], center[1], center[2]);
    glRotatef(angles[0] * 180.0/math.pi, 1, 0, 0)
    glRotatef(angles[1] * 180.0/math.pi, 0, 1, 0)
    glRotatef(angles[2] * 180.0/math.pi, 0, 0, 1)
    #print(center)
    #print(angles)
    color = [1.0,0.,0.,1.]
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    glutWireTeapot(1,2,2)
    glPopMatrix()
    
    
    glPushMatrix()
    glTranslatef(center2[0], center2[1], center2[2]);
    glRotatef(phi2, 1, 0, 0)
    glRotatef(theta2, 0, 1, 0)
    glRotatef(psi2, 0, 0, 1)
    color = [1.0,0.,0.,1.]
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    glutWireTeapot(1,2,2)
    glPopMatrix()
        
    
    glutSwapBuffers()
    
    return

def on_keyboard(key, x, y):
    
    global animation_ongoing
    
    if(key == 'g' or key == 'G'):
        if(animation_ongoing == 0):
     	  glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID)
     	  animation_ongoing=1
    


def on_timer(value):
    
    global t
    global tm
    global animation_ongoing

    if (value != TIMER_ID):
        return
    
    if(t < tm - 0.1):
        t = t + 0.1
    else:
        animation_ongoing = 0
    
    glutPostRedisplay()
    
    if (animation_ongoing):
        glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
     
    

def draw_coordsys():
    
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    
    glColor3f (1, 0, 0);
    glVertex3f(150, 0, 0);
    glVertex3f(0, 0, 0);

    glColor3f (0, 1, 0);
    glVertex3f(0, 150, 0);
    glVertex3f(0, 0, 0);
    
    glColor3f (0, 0, 1);
    glVertex3f(0, 0, 150);
    glVertex3f(0, 0, 0);
    glEnd();
    glEnable(GL_LIGHTING);


if __name__ == '__main__': main()