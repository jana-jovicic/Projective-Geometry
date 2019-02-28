import numpy as np
import math
import sys

def Euler2A(phi, theta, psi):
    
    Rx = np.empty([3, 3])
    Rx[0] = [1, 0, 0]
    Rx[1] = [0, math.cos(phi), -math.sin(phi)]
    Rx[2] = [0, math.sin(phi), math.cos(phi)]
    
    Ry = np.empty([3, 3])
    Ry[0] = [math.cos(theta), 0, math.sin(theta)]
    Ry[1] = [0, 1, 0]
    Ry[2] = [-math.sin(theta), 0, math.cos(theta)]
    
    Rz = np.empty([3, 3])
    Rz[0] = [math.cos(psi), -math.sin(psi), 0]
    Rz[1] = [math.sin(psi), math.cos(psi), 0]
    Rz[2] = [0, 0, 1]
    
    tmp = np.matmul(Rz, Ry)
    return np.matmul(tmp, Rx)


def AxisAngle(A):
    
    #odrediti jedinicni sopstveni vektor p za lambda = 1 ;
    E = [[1,0,0], [0,1,0], [0,0,1]]
    M = A - E
    p = np.cross(M[0], M[1])    # vektorski proizvod
    normP = np.linalg.norm(p, 2)
    p = p/normP
    
    #odrediti proizvoljan jedinicni vektor u normalan na p;
    u = M[1]
    # u' = Au;  // u' je jedinicni;
    u2 = np.matmul(A, u)
    tmp = np.dot(u, u2) # skalarni proizvod
    tmp = tmp / (np.linalg.norm(u, 2) * np.linalg.norm(u2, 2))
    phi = math.acos(tmp)
    
    M2 = np.empty([3,3])
    M2[0] = u
    M2[1] = u2
    M2[2] = p
    
    if(np.linalg.det(M2) < 0):
        p = -p
        
    return p, phi



def Rodrigez(p, phi):
    
    pT = np.array([[p[0]], [p[1]], [p[2]]])
    tmp1 = p*pT
    
    E = [[1,0,0], [0,1,0], [0,0,1]]
    tmp2 = math.cos(phi) * (E - tmp1)
    
    px = np.empty([3,3])
    px[0] = [0, -p[2], p[1]]
    px[1] = [p[2], 0, -p[0]]
    px[2] = [-p[1], p[0], 0]
    
    R = tmp1 + tmp2 + math.sin(phi)*px
    return R
    
    
def A2Euler(a):
    
    if(a[2][0] < 1):
        if(a[2][0] > -1):
            psi = math.atan2(a[1][0], a[0][0])
            theta = math.sin(-a[2][0])
            phi = math.atan2(a[2][1], a[2][2])
        else:
            psi = math.atan2(-a[0][1], a[1][1])
            theta = math.pi/2
            phi = 0
    else:
        psi = math.atan2(-a[0][1], a[1][1])
        theta = - math.pi/2
        phi = 0
        
    return [phi, theta, psi]


def AxisAngle2Q(p, phi):

    w = math.cos(phi/2.0)
    p = p / np.linalg.norm(p, 2)
    sin = math.sin(phi/2.0)
    x = sin * p[0]
    y = sin * p[1]
    z = sin * p[2]
    return [x, y, z, w]


def Q2AxisAngle(q):
    
    q = q / np.linalg.norm(q, 2)
    w = q[3]
    
    if(w < 0):
        q = -q
        
    phi = 2 * math.acos(q[3])
    
    if(w == 1 or w == -1):
        p = [1, 0, 0]
    else:
        tmp = [q[0], q[1], q[2]]
        p = tmp / np.linalg.norm(tmp, 2)
        
    return p, phi


    
def main():
    
    print("Unesite phi:")
    phi = input()
    print("Unesite theta:")
    theta = input()
    print("Unesite psi:")
    psi = input()
    
    print("angles in decimal:")
    print([phi, theta, psi])
    
    A = Euler2A(phi, theta, psi)
    print("Euler2A:")
    print(A)
    print("")
    
    print("AxisAngle:")
    p, angle = AxisAngle(A)
    sys.stdout.write("p = ")
    print(p)
    sys.stdout.write("angle = ")
    print(angle)
    print("")
    
    print("Rodrigez:")
    rotationMatrix = Rodrigez(p, angle)
    print(rotationMatrix)
    print("")
    
    print("A2Euler:")
    angles = A2Euler(A)
    print(angles)
    print("")
    
    print("AxisAngle2Q:")
    q = AxisAngle2Q(p, angle)
    print(q)
    print("")
    
    print("Q2AxisAngle:")
    p, angle = Q2AxisAngle(q)
    sys.stdout.write("p = ")
    print(p)
    sys.stdout.write("angle = ")
    print(angle)
    
if __name__ == "__main__":
    main()