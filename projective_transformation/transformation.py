import matplotlib.pyplot as plt
import numpy as np

points = []
pointImages = []
numOfPoints = 0;
n = 0;

def naiveFindP(pointMatrix, num):
    U = np.matrix(pointMatrix)
    U = np.delete(U, 3, 0)      # deleting last row (last point)
    Utranspose = np.transpose(U)
    Uinverse = np.linalg.inv(Utranspose)
    d = pointMatrix[num-1]   # last point
    
    lambdas = Uinverse.dot(d)
    lambdasT = np.transpose(lambdas)
    
    Ptmp = np.empty([3, 3])
    for i in range(3):
        for j in range(3):
            Ptmp[j][i] = pointMatrix[j][i] * lambdasT[j]
    P = np.transpose(Ptmp)
    return P
    
def naive(num):
    global n
    global points
    global pointImages
        
    P1tmp = naiveFindP(points, num)
    P1 = np.linalg.inv(P1tmp)
    P2 = naiveFindP(pointImages, num)
    P = np.matmul(P2, P1)
    return P


def create2x9Matrix(point, pointP):
    a = np.empty([2, 9])
    a[0] = [0, 0, 0, -1*pointP[2]*point[0], -1*pointP[2]*point[1], -1*pointP[2]*point[2], pointP[1]*point[0], pointP[1]*point[1], pointP[1]*point[2]]
    a[1] = [pointP[2]*point[0], pointP[2]*point[1], pointP[2]*point[2], 0, 0, 0, -1*pointP[0]*point[0], -1*pointP[0]*point[1], -1*pointP[0]*point[2]]
    return a;
    
def DLT(points, pointImages):
    
    A = create2x9Matrix([0,0,0], [0,0,0])
    for i in range(numOfPoints):
        a = create2x9Matrix(points[i], pointImages[i])
        A = np.concatenate((A, a))
    A = np.delete(A, 0, 0)
    A = np.delete(A, 0, 0)
    
    u, d, v = np.linalg.svd(A, full_matrices=True)
    P = v[len(v)-1, :]
    P = np.reshape(P, (3, 3))
    return P


def normalize(points):
    # calculating the center of point system
    T = np.array([0,0,0])
    for p in points:
        T = T + p
    T = T / numOfPoints
    # translating T in origin
    G = np.array([[1, 0, -T[0]], [0, 1, -T[1]], [0, 0, 1]])
    # translating points whith G
    translatedPoints = np.empty([numOfPoints, 3])
    for i in range(numOfPoints):
        translatedPoints[i] = np.matmul(G, points[i])
    # calculating Euclidean distance of translated points and origin
    distance = np.empty([numOfPoints])
    for i in range(numOfPoints):
        distance[i] = np.sqrt(translatedPoints[i][0]**2 + translatedPoints[i][1]**2)
    # calculating average distance
    avgDist = 0
    for d in distance:
        avgDist = avgDist + d
    avgDist = avgDist / numOfPoints
    # calculating scaling matrix
    scale = np.sqrt(2) / avgDist
    S = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1]])
    # calculating normalization matrix
    N = np.matmul(S, G)
    return N
    
def normalizedDLT(num):
    T = normalize(points)
    normalizedPoints = np.empty([numOfPoints, 3])
    for i in range(numOfPoints):
        normalizedPoints[i] = np.matmul(T, points[i])
        
    TP = normalize(pointImages)
    normalizedPointImages = np.empty([numOfPoints, 3])
    for i in range(numOfPoints):
        normalizedPointImages[i] = np.matmul(TP, pointImages[i])
    TPinverse = np.linalg.inv(TP)
        
    P_ = DLT(normalizedPoints, normalizedPointImages)
    P = np.matmul(np.matmul(TPinverse, P_), T)
    return P


def main():
    
    global numOfPoints
    global n
    global points
    global pointImages
    
    fig, ax = plt.subplots()
    plt.axis([-10, 10, -10, 10])
    plt.xlabel('x')
    plt.ylabel('y')
    
    numOfPoints = int(raw_input("Enter number of points: "))
    fileName = raw_input("Enter name of file that contains points: ")
    
    j = 0
    
    try:
        with open(fileName) as f:
            for line in f:
                x1, x2, x3 = [float(x) for x in line.split()]
                if j < numOfPoints:
                    points.append([x1, x2, x3])
                    if j > 0:
                        plt.plot([x1,points[j-1][0]], [x2,points[j-1][1]], color='blue')
                    if j == numOfPoints-1:
                        plt.plot([x1,points[0][0]], [x2,points[0][1]], color='blue')
                else:
                    pointImages.append([x1,x2,x3])
                    if j > numOfPoints:
                        plt.plot([x1,pointImages[j-numOfPoints-1][0]], [x2,pointImages[j-numOfPoints-1][1]], color='green')
                    if j == 2*numOfPoints-1:
                        plt.plot([x1,pointImages[0][0]], [x2,pointImages[0][1]], color='green')
                j = j+1
                if(j <= numOfPoints):
                    plt.scatter(x1, x2, color='blue')
                else:
                    plt.scatter(x1, x2, color='green')
    except IOError:
        print("File open failed")
    
    print("Points:")
    for p in points:
        print(p)
        
    print("Images of points: ")
    for p in pointImages:
        print(p)
            
    if(numOfPoints == 4):
        Pn = naive(numOfPoints)
        print("Naive algorithm:")
        print(Pn)
        Pdlt = DLT(points, pointImages)
        print("DLT algorithm:")
        print(Pdlt)
        Pndlt = normalizedDLT(numOfPoints)
        print("Normalized DLT algorithm::")
        print(Pndlt)
    else:
        Pdlt = DLT(points, pointImages)
        print("DLT algorithm:")
        print(Pdlt)
        Pndlt = normalizedDLT(numOfPoints)
        print("Normalized DLT algorithm::")
        print(Pndlt)
    
    
    plt.show()
    
    
if __name__ == "__main__":
    main()
