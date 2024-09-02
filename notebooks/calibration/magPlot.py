
from matplotlib import pyplot as plt
from scipy import linalg
from collections import deque
import sys
import time
import numpy as np
from math import radians as deg2rad
from math import degrees as rad2deg

class MagPlot:

    ok = True

    def __init__(self, BUFFER_SIZE=10000):
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.set_aspect(1)

        self.mag_x = deque(maxlen=BUFFER_SIZE)
        self.mag_y = deque(maxlen=BUFFER_SIZE)
        self.mag_z = deque(maxlen=BUFFER_SIZE)

        self.fig.canvas.mpl_connect('key_press_event', self.on_press)
        self.epoch = time.monotonic()
        self.count = 0

    def on_press(self, event):
        print("press: ", event.key)
        sys.stdout.flush()
        if event.key in ['q', 'ctrl+c']:
            self.ok = False

    def push(self, x,y,z):
        # save data for real-time plotting
        self.mag_x.append(x)
        self.mag_y.append(y)
        self.mag_z.append(z)

        self.count += 1
        if (self.count % 100 == 0):
            now = time.monotonic()
            hz = self.count / (now - self.epoch)
            print(f"{hz:4.1f}", end="\r")
            sys.stdout.flush()
            self.epoch = now
            self.count = 0

    def plot(self, title=None):
        # Clear all axis
        self.ax.cla()
        # self.ax.clear()

        x = self.mag_x
        y = self.mag_y
        z = self.mag_z

        # Display the sub-plots
        self.ax.scatter(x, y, color='r', label="X-Y")
        self.ax.scatter(y, z, color='g', label="Y-Z")
        self.ax.scatter(z, x, color='b', label="Z-X")
        self.ax.grid()
        self.ax.legend(loc="upper left")

    def pause(self, interval=0.001):
        # self.fig.canvas.start_event_loop(interval)
        plt.pause(interval)


def plotMagnetometer(data, title=None):
    x = [v[0] for v in data]
    rx = (max(x)-min(x))/2
    cx = min(x)+rx

    y = [v[1] for v in data]
    ry = (max(y)-min(y))/2
    cy = min(y)+ry

    z = [v[2] for v in data]
    rz = (max(z)-min(z))/2
    cz = min(z)+rz

    alpha = 0.5
    u = np.linspace(0, 2 * np.pi, 100)

    plt.plot(rx*np.cos(u)+cx, ry*np.sin(u)+cy,'-r',label='xy')
    plt.plot(x,y,'.r',alpha=alpha)

    plt.plot(rx*np.cos(u)+cx, rz*np.sin(u)+cz,'-g',label='xz')
    plt.plot(x,z,'.g',alpha=alpha)

    plt.plot(rz*np.cos(u)+cz, ry*np.sin(u)+cy,'-b',label='zy')
    plt.plot(z,y, '.b',alpha=alpha)

    plt.title(f"CM:({cx:.1f}, {cy:.1f}, {cz:.1f}) uT  R:({rx:.1f}, {ry:.1f}, {rz:.1f}) uT")
    plt.xlabel('$\mu$T')
    plt.ylabel('$\mu$T')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()


def calibrate(s, F=1):
    ''' Performs calibration of magnetometer data by performing an
    elliptical fit of the data.

    Qingde Li and John G.Griffiths, Least Squares Ellipsoid Specific Fitting, DOI: 10.1109/GMAP.2004.1290055 · Source: IEEE Xplore


    F: expected field strength, default is 1
    Returns: an A and b matrix, such that, actual_mag = A*noisy_mag + b
    '''
    s = s.T

    # D (samples)
    D = np.array([
        s[0]**2., s[1]**2., s[2]**2.,
        2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
        2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1,  1,  1,  0,  0,  0],
                  [ 1, -1,  1,  0,  0,  0],
                  [ 1,  1, -1,  0,  0,  0],
                  [ 0,  0,  0, -4,  0,  0],
                  [ 0,  0,  0,  0, -4,  0],
                  [ 0,  0,  0,  0,  0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(
        linalg.inv(C),
        S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    E_w, E_v = np.linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

    # quadric-form parameters
    M = np.array([[v_1[0], v_1[3], v_1[4]],
                  [v_1[3], v_1[1], v_1[5]],
                  [v_1[4], v_1[5], v_1[2]]])
    n = np.array([[v_2[0]],
                  [v_2[1]],
                  [v_2[2]]])
    d = v_2[3]

    #-------------------------------------
    # calibration parameters
    # note: some implementations of sqrtm return complex type, taking real
    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n).T[0]  # make numpy array [bx,by,bz]

    A = F / np.sqrt(np.abs(np.dot(n.T, np.dot(M_1, n)) - d))
    A = A*np.real(linalg.sqrtm(M))

    return A, b

def magcal(Bp, uT=None):
    """
    Modelled after the matlab function: magcal(D) -> A, b, expmfs
    inputs:
      Bp: data points
      uT: expected field strength for longitude/altitude. If None
          is given, then automatically calculated and used
    returns:
      A: soft-iron 3x3 matrix of scaling
      b: hard-iron offsets
      expmfs: expected field strength"""
    Y = np.array([v[0]**2+v[1]**2+v[2]**2 for v in Bp])
    X = np.hstack((Bp,np.ones((Bp.shape[0],1))))
    beta = np.linalg.inv(X.T.dot(X)).dot(X.T.dot(Y))
    b=0.5*beta[:3]

    # expected mag field strength
    expmfs=np.sqrt(beta[3]+b[0]**2+b[1]**2+b[2]**2)

    if uT is None:
        uT = expmfs

    x = [v[0] for v in Bp]
    rx = (max(x)-min(x))/2

    y = [v[1] for v in Bp]
    ry = (max(y)-min(y))/2

    z = [v[2] for v in Bp]
    rz = (max(z)-min(z))/2

    A = np.diag([uT/rx,uT/ry,uT/rz])
    return A,b,expmfs

def another_way(Bpp):
    """
    Where does this come from?

    D = [X2, Y2, Z2, 2XY,2XZ, 2YZ, 2X, 2Y, 2Z]
    v = [ a, b, c, d, e, f, g, h, i ]
    1 [Nx1]
    v[9x1] = inv(DT D)[9x9] (DT 1)[9x1]

    A = [
        v(1) v(4) v(5) v(7);
        v(4) v(2) v(6) v(8);
        v(5) v(6) v(3) v(9);
        v(7) v(8) v(9) -1 ];
    ofs=-A(1:3,1:3)\[v(7);v(8);v(9)]; % offset is center of ellipsoid
    Tmtx=eye(4);
    Tmtx(4,1:3)=ofs';
    AT=Tmtx*A*Tmtx'; % ellipsoid translated to (0,0,0)
    [rotM ev]=eig(AT(1:3,1:3)/-AT(4,4)); % eigenvectors (rotation) and eigenvalues (gain)
    gain=sqrt(1./diag(ev)); % gain is radius of the ellipsoid
    """
    x = Bpp[:,0]
    y = Bpp[:,1]
    z = Bpp[:,2]
    sz = len(Bpp)
    D = np.array([x*x,y*y,z*z,2*x*y,2*x*z,2*y*z,2*x,2*y,2*z]).T
    v = np.linalg.inv(D.T @ D) @ (D.T @ np.ones(sz))
    print(v)
    A = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8],   -1]
    ])

    ofs=-np.linalg.inv(A[:3,:3]) @ np.array([v[6], v[7], v[8]]).T
    Tmtx = np.eye(4)
    Tmtx[3,:3] = ofs
    AT = Tmtx @ A @ Tmtx.T
    ev, rotM = np.linalg.eig(AT[:3,:3]/-AT[3,3])
    gain = np.sqrt(1/ev)  # added abs, negative sqrt values

    print(f"gain: {gain}")
    print(f"offsets: {ofs}")
    print(f"rot mat: \n {rotM}")
    return gain, ofs, rotM