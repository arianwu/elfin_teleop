import numpy as np
from copy import copy

pi = np.pi


def dh(theta, d, alpha, a):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.

    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T


def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]

    """
    T1 = dh(q[0]          , 0.22,  np.pi/2, 0   )
    T2 = dh(q[1] + np.pi/2, 0   ,  np.pi  , 0.38)
    T3 = dh(q[2] + np.pi/2, 0   ,  np.pi/2, 0   )
    T4 = dh(q[3] + np.pi  , 0.42,  np.pi/2, 0   )
    T5 = dh(q[4]          , 0   , -np.pi/2, 0   )
    T6 = dh(q[5] + np.pi  , 0.18,  0      , 0   )
    pose = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return pose


def fkine_pos(q):
    T = fkine(q)
    return T[0:3,3]


def jacobian_position(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((3,q.size))
    init_fq = fkine_pos(q)
    for k in range(q.size):

        deltaq = np.copy(q)
        deltaq[k] += delta

        x_delta = (fkine_pos(deltaq) - init_fq)/delta

        J[0:3,k] = x_delta.transpose()
        
    rank = np.linalg.matrix_rank(J)

    return J


def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,q.size))
    init_pos = fkine(q)[0:3,3]
    init_rot = fkine(q)[0:3,0:3]
    init_quat = rot2quat(init_rot)

    for k in range(q.size):
        deltaq = np.copy(q)
        deltaq[k] += delta

        x_delta = (fkine(deltaq)[0:3,3] - init_pos)/delta

        quat_delta = (rot2quat(fkine(deltaq)[0:3,0:3]) - init_quat)/delta

        J[0:3,k] = x_delta.transpose()

        J[3:8,k] = quat_delta.transpose()

    rank = np.linalg.matrix_rank(J)

    return J


def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R
