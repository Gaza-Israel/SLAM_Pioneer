import EKF_v4 as EKF_v4
from ctypes import sizeof
import numpy as np
from random import randint
from random import seed
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython import display
from filterpy.stats import plot_covariance
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


class landmark():
    def __init__(self, landmarks):
        self.landmark_g = np.array(landmarks.copy()).astype(float) 
        self.landmark_r = np.array(landmarks.copy()).astype(float)
        self.landmark_noise_r = np.array(landmarks.copy()).astype(float)
        self.landmark_noise_g = np.array(landmarks.copy()).astype(float)

# seed(10)
l = []
dmax = 20
for i in range(0, 20):
    a1 = randint(-10, 10)
    a2 = randint(-1, 10)
    l.append([a1, a2])
l = np.array(l) 

# l = np.array([[5,5], [0,5], [14,10], [-5,10], [-10,10]]) # Vetor com a posição de todas as landmarks no frame global 
n = len(l) # number of landmarks 

# inicializa o estado inicial com n*2 landmark e pose (3)
x0 = np.zeros(n*2 + 3)
x0[0] = 0
x0[1] = 0
x0[2] = np.deg2rad(0)

# Inicializar as incertezas 
sigma0 = np.identity(n*2 + 3) * 0
sigma0[0][0] = 0
sigma0[1][1] = 0
sigma0[2][2] = 0

Q = np.identity(2) # uncertanty in the measurement, bearing and range 
Q[0][0] = (0.5)**2
Q[1][1] = np.deg2rad((5)**2) 

u = [1, np.deg2rad(10)]
dt = 0.5

mean = 0
std = 1e-2
num_samples = 2
np.random.seed(10) 
noise_fact = 2e-2

np.set_printoptions(suppress=True, formatter={'float_kind':'{:16.5f}'.format}, linewidth=400)

## Parte para plotar 
# create empty lists for the x and y data
ekf_x = []
ekf_y = []

x_c = []
y_c = []
x_n = []
y_n = []

l_xc = []
l_yc = []
l_xn = []
l_yn = []

# create the figure and axes objects
fig, ax = plt.subplots()

controled_ekf = EKF_v4.modelo(x0, dt, sigma0, True)
noise_ekf = EKF_v4.modelo(x0, dt, sigma0, True)
real_ekf_model = EKF_v4.modelo(x0, dt, sigma0, False)
real_ekf_filter = EKF_v4.EKF()


land = landmark(l)

def gera_sinal_landmark(ekf, land):
    # a partir da posicao atual do ekf, e da posicao global das landmarks, gera a posicao relativa entre as landmarks e o robo, com ruido
    x = ekf.x[0]
    y = ekf.x[1]
    theta = ekf.x[2]
    k = 0

    for i in land.landmark_g:
        lx = i[0]
        ly = i[1]

        c = np.cos(-theta) 
        s = np.sin(-theta)
        R_frame = np.array([[c, -s], [s, c]])
        noise_land = np.array(np.random.normal(mean, std, size=num_samples))
        noise_land = noise_land.reshape(1,2)
                
        land.landmark_r[k] = np.array(land.landmark_g[k] @ R_frame.T)
        land.landmark_noise_g[k] = land.landmark_g[k] + noise_land
        land.landmark_noise_r[k] = np.array(land.landmark_noise_g[k] @ R_frame.T)
    
        k = k + 1

def closest_landmarks(modelo, land, dmax):
    x = modelo.x[0]
    y = modelo.x[1]
    lands_x = []
    lands_y = []
    k = 0
    indices_lands = []
    lands_proximas = []    

    for i in land.landmark_noise_g:
        dist = np.sqrt((i[0] - x)**2 + (i[1] - y)**2)
        if dist < dmax:
            lands_x.append(i[0]) 
            lands_y.append(i[1])
            indices_lands.append(k)
            lands_proximas.append(i)
        k = k + 1
        
    return(lands_x, lands_y, indices_lands, lands_proximas)
    


def add_noise(ekf):
    ekf.x[0] = ekf.x[0] + randint(-100,100)/100 * noise_fact 
    ekf.x[1] = ekf.x[1] + randint(-100,100)/100 * noise_fact
    ekf.x[2] = ekf.x[2] + randint(-100,100)/100 * noise_fact

def animate(i, u, c_ekf, n_ekf, real_ekf_model, real_ekf_filter, land):
    
    c_ekf.move(u)

    add_noise(n_ekf)
    n_ekf.move(u)

    gera_sinal_landmark(real_ekf_model, land)

    closest_x, closest_y , indices_lands, lands_proximas = (closest_landmarks(real_ekf_model, land, dmax))

    real_ekf_filter.correct_prediction(Q, real_ekf_model, lands_proximas, indices_lands)
    real_ekf_model.move(u)
    # print(real_ekf_model.x)
    # real_ekf_filter.correct_prediction(Q, real_ekf_model, land.landmark_noise_g, range(0,n+1))

    # Landmarks com ruido 
    l_xn = land.landmark_noise_g[:,0]
    l_yn = land.landmark_noise_g[:,1]

    # Landmarks sem ruido 
    l_xc = land.landmark_g[:,0]
    l_yc = land.landmark_g[:,1]

    # Plota a posicao do robo controlado
    x_c.append(c_ekf.x[0])
    y_c.append(c_ekf.x[1])

    # Plota a posicao do robo com ruido
    x_n.append(n_ekf.x[0])
    y_n.append(n_ekf.x[1])

    # Plota a posicao do robo com ekf
    ekf_x.append(real_ekf_model.x[0])
    ekf_y.append(real_ekf_model.x[1])

    ax.clear()

    ax.plot(x_c, y_c, label = 'Controled trajectory', linestyle='dashed')
    # ax.plot(x_n, y_n, label = 'noise')
    ax.plot(ekf_x, ekf_y, label = 'EKF trajectory')

    ax.scatter(l_xc, l_yc, label = 'Landmarks', marker = 's')
    # ax.scatter(l_xn, l_yn, label = 'landmark noise', marker = 's')

    ax.scatter(closest_x, closest_y, label = 'Landmark used', marker = 's')

    ax.set_xlim([-20,20])
    ax.set_ylim([-2,20])
    ax.set_xlabel('x coordinate [m]')
    ax.set_ylabel('y coordinate [m]')
    ax.legend()

    # plot_covariance((real_ekf_model.x[0], real_ekf_model.x[1]), real_ekf_model.sigma, std=6, facecolor='g', alpha=0.8, axes = ax)
    ax.grid()

# run the animation
ani = FuncAnimation(fig, animate, fargs=(u, controled_ekf, noise_ekf, real_ekf_model, real_ekf_filter, land), frames=80, interval=100, repeat=False)
ani.save('a.gif', writer='imagemagick', fps=30)
# plt.show()

