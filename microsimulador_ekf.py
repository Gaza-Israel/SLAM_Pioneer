import EKF_v3 as EKF_v3
from ctypes import sizeof
import numpy as np
from random import randint
from random import seed
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython import display

class landmark():
    def __init__(self, landmarks):
        self.landmark_g = np.array(landmarks.copy()).astype(np.float) 
        self.landmark_r = np.array(landmarks.copy()).astype(np.float)
        self.landmark_noise_r = np.array(landmarks.copy()).astype(np.float)
        self.landmark_noise_g = np.array(landmarks.copy()).astype(np.float)


plt.close('all')

# l = np.array([[5,5], [0,5], [15,10], [-5,10], [-10,10]]) # Vetor com a posição de todas as landmarks no frame global 
l = np.array([[5,5], [7,7], [10,10]]) # Vetor com a posição de todas as landmarks no frame global 
n = len(l) # number of landmarks 

# inicializa o estado inicial com n*2 landmark e pose (3)
x0 = np.zeros(n*2 + 3)
x0[0] = 0
x0[1] = 0
x0[2] = 0

# Inicializar as incertezas 
sigma0 = np.identity(n*2 + 3) * 1e20
sigma0[0][0] = 0
sigma0[1][1] = 0
sigma0[2][2] = 0

infinito = 1e20
Q = np.identity(2) # uncertanty in the measurement, bearing and range 
Q[0][0] = 1e-2
Q[1][1] = 1e-2

u = [0.5, np.deg2rad(3)]
dt = 1

mean = 0
std = 1e-2
num_samples = 2
np.random.seed(10) 
noise_fact = 1e-10

# teste = ekf.modelo(x0, dt, sigma0)
# teste_ekf = ekf()
# obs = np.array([[2, 2], [1,1]])
# teste_ekf.correct_prediction(Q, teste, obs, [1, 2])
# print(teste.x)

np.set_printoptions(suppress=True, formatter={'float_kind':'{:16.10f}'.format}, linewidth=250)
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

controled_ekf = EKF_v3.modelo(x0, dt, sigma0, True)
noise_ekf = EKF_v3.modelo(x0, dt, sigma0, True)
real_ekf_model = EKF_v3.modelo(x0, dt, sigma0, False)
real_ekf_filter = EKF_v3.EKF()

# l = np.array([[3,3]]) # Vetor com a posição de todas as landmarks no frame global 

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
        land.landmark_noise_g[k] = land.landmark_g[k] +noise_land
        land.landmark_noise_r[k] = np.array(land.landmark_noise_g[k] @ R_frame.T)
    
        k = k + 1

def add_noise(ekf):
    ekf.x[0] = ekf.x[0] + randint(-100,100)/100 * noise_fact 
    ekf.x[1] = ekf.x[1] + randint(-100,100)/100 * noise_fact
    ekf.x[2] = ekf.x[2] + randint(-100,100)/100 * noise_fact

def animate(i, u, c_ekf, n_ekf, real_ekf_model, real_ekf_filter, land):

    c_ekf.move(u)

    # add_noise(n_ekf)
    # n_ekf.move(u)
    gera_sinal_landmark(real_ekf_model, land)
    real_ekf_model.move(u)
    real_ekf_filter.correct_prediction(Q, real_ekf_model, land.landmark_noise_g, range(0,n+1))

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

    ax.plot(x_c, y_c, label = 'controled', linestyle='dashed')
    # ax.plot(x_n, y_n, label = 'noise')
    ax.plot(ekf_x, ekf_y, label = 'EKF')

    ax.scatter(l_xc, l_yc, label = 'landmark controled', marker = 's')
    ax.scatter(l_xn, l_yn, label = 'landmark noise', marker = 's')

    ax.set_xlim([-20, 30])
    ax.set_ylim([-20, 30])
    ax.set_xlabel('x coordinate [m]')
    ax.set_ylabel('y coordinate [m]')
    ax.legend()

    ax.grid()

# run the animation
ani = FuncAnimation(fig, animate, fargs=(u, controled_ekf, noise_ekf, real_ekf_model, real_ekf_filter, land), frames=150, interval=50, repeat=False)
ani.save('myAnimation.gif', writer='imagemagick', fps=30)
# plt.show()

