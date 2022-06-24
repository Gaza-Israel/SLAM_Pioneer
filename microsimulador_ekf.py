import EKF_v3 as EKF_v3
from ctypes import sizeof
import numpy as np
from random import randint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython import display

class landmark():
    def __init__(self, landmarks):
        self.landmark_g = landmarks.copy() 
        self.landmark_r = landmarks.copy()
        self.landmark_noise_r = landmarks.copy()
        self.landmark_noise_g = landmarks.copy()


plt.close('all')

n = 3 # number of landmarks 

# inicializa o estado inicial com n*2 landmark e pose (3)
x0 = np.zeros(n*2 + 3)
x0[0] = 0
x0[1] = 0
x0[2] = 0
noise_fact = 0

# Inicializar as incertezas 
sigma0 = np.identity(n*2 + 3) * 1e20
sigma0[0][0] = 0
sigma0[1][1] = 0
sigma0[2][2] = 0

infinito = 1e20
Q = np.identity(2) # uncertanty in the measurement, bearing and range 
Q[0][0] = infinito
Q[1][1] = infinito

u = [1,0.1]
dt = 0.5

# teste = ekf.modelo(x0, dt, sigma0)
# teste_ekf = ekf()
# obs = np.array([[2, 2], [1,1]])
# teste_ekf.correct_prediction(Q, teste, obs, [1, 2])
# print(teste.x)

np.set_printoptions(suppress=True, formatter={'float_kind':'{:16.3f}'.format}, linewidth=100)
# teste = ekf.modelo(x0, dt, sigma0)

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
# function that draws each frame of the animation

controled_ekf = EKF_v3.modelo(x0, dt, sigma0)
noise_ekf = EKF_v3.modelo(x0, dt, sigma0)
real_ekf_model = EKF_v3.modelo(x0, dt, sigma0)
real_ekf_filter = EKF_v3.EKF()

l = np.array([[1,2], [3,4], [5,6]]) # Vetor com a posição de todas as landmarks no frame global 
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

        land.landmark_r[k] = land.landmark_g[k] @ R_frame.T

        land.landmark_noise_g[k] = land.landmark_g[k] + np.array([randint(-100,100), randint(-100,100)]) * noise_fact
        land.landmark_noise_r[k] = land.landmark_noise_g[k] @ R_frame.T

        k = k + 1

def add_noise(ekf):
    ekf.x[0] = ekf.x[0] + randint(-100,100) * noise_fact 
    ekf.x[1] = ekf.x[1] + randint(-100,100) * noise_fact
    ekf.x[2] = ekf.x[2] + randint(-100,100) * noise_fact

def animate(i, u, c_ekf, n_ekf, real_ekf_model, real_ekf_filter, land):

    c_ekf.move(u)

    add_noise(n_ekf)
    n_ekf.move(u)

    gera_sinal_landmark(c_ekf, land)
    real_ekf_filter.correct_prediction(Q, real_ekf_model, land.landmark_noise_r, [0, 1, 2])
    real_ekf_model.move(u)

    l_xn = land.landmark_noise_g[:,0]
    l_yn = land.landmark_noise_g[:,1]

    l_xc = land.landmark_g[:,0]
    l_yc = land.landmark_g[:,1]

    # Plota a posicao do robo controlado
    x_c.append(c_ekf.x[0])
    y_c.append(c_ekf.x[1])

    # Plota a posicao do robo com ruido
    x_n.append(n_ekf.x[0])
    y_n.append(n_ekf.x[1])

    # Plota a posicao do robo controlado
    ekf_x.append(real_ekf_model.x[0])
    ekf_y.append(real_ekf_model.x[1])

    ax.clear()

    ax.plot(x_c, y_c, label = 'controled')
    ax.plot(x_n, y_n, label = 'noise')
    ax.plot(ekf_x, ekf_y, label = 'EKF')

    ax.scatter(l_xc, l_yc, label = 'landmark controled', marker = 's')
    ax.scatter(l_xn, l_yn, label = 'landmark noise', marker = 's')

    ax.set_xlim([-20,20])
    ax.set_ylim([-10,30])
    ax.set_xlabel('x coordinate [m]')
    ax.set_ylabel('y coordinate [m]')
    ax.legend()

    ax.grid()

# run the animation
ani = FuncAnimation(fig, animate, fargs=(u, controled_ekf, noise_ekf, real_ekf_model, real_ekf_filter, land), frames=130, interval=50, repeat=False)
ani.save('myAnimation.gif', writer='imagemagick', fps=30)
# plt.show()


