from ctypes import sizeof
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

class EKF():
    def __init__(self):
        self.H = 0                  # Jacobian for the observation
        self.z_predict = 0          # Predicted location of the given landmark 
        self.K = 0                  # Kalmann Gain
        self.conta_landmarks = 0    # Contador do numero de landmarks ja observadas
        self.r_measure = 0          # Raio da medicao (range)
        self.phi_measure = 0        # Angulo da medicao (bearing)

    def correct_prediction(self, Q, modelo, landmarks, idx_landmarks): # Corrige o vetor de estados e covariancias com base no ganho de kalman 
        k = 0
        for i in landmarks:     # Itera para todas as landamarks sendo observadas atualmente 
            j = idx_landmarks[k] 
            self.obs_predict(j, landmarks[k], modelo)   # Preve a posicao das landmarks 
            self.KalmanGain(modelo, Q)                  # Calcula o ganho de Kalman
            land_range_bearing = np.array([self.r_measure, self.phi_measure])
            modelo.x_estimate = modelo.x_estimate + self.K @ -(land_range_bearing - self.z_predict)
            l,m = ((self.K @ self.H).shape)
            I = np.identity(l)
            modelo.sigma_estimate = (I - self.K @ self.H) @ modelo.sigma_estimate
            k = k + 1

        modelo.sigma = modelo.sigma_estimate
        modelo.x = modelo.x_estimate

    def obs_predict(self, j, land_pos, modelo): 
        # Recebe a posicao das features ja no frame global, e faz uma predicao de onde eu deveria observar essa landmark 
        n = len(modelo.x_estimate)
        x = modelo.x_estimate[0]
        y = modelo.x_estimate[1]
        theta = modelo.x_estimate[2]
        
        j = int(j+1)
        land_x = land_pos[0] 
        land_y = land_pos[1]

        if (j > self.conta_landmarks):
            # Se a landmark nunca foi observada, sua posicao e a propria observacao 
            pos_land = np.array([land_x, land_y])
            modelo.x[3 + 2*(j-1)] = land_x
            modelo.x[4 + 2*(j-1)] = land_y
            modelo.x_estimate[3 + 2*(j-1)] = land_x
            modelo.x_estimate[4 + 2*(j-1)] = land_y 
            self.conta_landmarks = self.conta_landmarks + 1
        else:
            # Se ela ja foi observada, sua posicao e a que esta salva no vetor de estados 
            land_x_antigo = modelo.x[3 + 2*(j-1)]
            land_y_antigo = modelo.x[4 + 2*(j-1)]
            pos_land = np.array([land_x_antigo, land_y_antigo])

        # Transformando a medida de cartesiano para range bearing
        deltax = (land_x - x)
        deltay = (land_y - y)
        self.r_measure  = np.sqrt((deltax)**2 + (deltay)**2)
        self.phi_measure  = np.arctan2(deltay, deltax) - theta
        pos_robo = np.array([x,y])
        delta = np.array(pos_land - pos_robo)
        q = delta.T @ delta
        self.z_predict = np.array([np.sqrt(q), np.arctan2(delta[1], delta[0]) - theta]) # prevendo a posicao da landmark  
        h = 1/q * np.array(([-np.sqrt(q) * delta[0], -np.sqrt(q) * delta[1], 0, np.sqrt(q) * delta[0], np.sqrt(q) * delta[0]], 
                    [delta[1], -delta[0], -q, -delta[1], delta[0]]))
       
        F = np.zeros((5,n))
        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1
        F[3][3 + 2*(j-1)] = 1
        F[4][4 + 2*(j-1)] = 1
        self.H = h @ F 

    def KalmanGain(self, modelo, Q):
        sigma = modelo.sigma
        H = (self.H)
        self.K = sigma @ (H.T) @ np.linalg.inv(H @ sigma @ (H.T) + Q)

class modelo():
    def __init__(self, x0, dt, sigma0, teste = False): # x = [x, y, theta], u = [velocity, angular_velocity] 
        self.x = x0 # State vector for the model, [x_coord, y_coord, theta, land_1_x, land_1_y, land_2_x, land_2_y ...]
        self.x_estimate = x0
        self.dt = dt # Time step 
        self.sigma = sigma0 # Covariances matrix 
        self.sigma_estimate = sigma0
        self.teste = teste
        

    def move(self, u): # updates de pose and covariance of the robot for a given control input 
        n = len(self.x_estimate)

        v = u[0]
        w = u[1]    
        theta = self.x[2]
        dt = self.dt

        if w == 0:
            w = 1e-10

        self.x_estimate[0] = -v/w * np.sin(theta) + v/w * np.sin(theta + w * dt) + self.x[0]
        self.x_estimate[1] = v/w * np.cos(theta) - v/w * np.cos(theta + w * dt) + self.x[1]
        self.x_estimate[2] = w * dt + self.x[2]
        
        if self.teste:
            self.x = self.x_estimate

        self.update_cov(u) 

    
 
    def update_cov(self, u): # updates de covariance matrix for a given input
        # This changes the uncertainty in the robot pose but not in the landmar position
        v = u[0]
        w = u[1]

        if w == 0:
            w = 1e-10

        theta = self.x_estimate[2]
        dt = self.dt
        n = len(self.x_estimate)

        Res = np.zeros((n,n)) #  User defined uncertainty in the range and bearing of the model 
        Res[0][0] = 1e-3
        Res[1][1] = 1e-3
        Res[2][2] = np.deg2rad(0.1)

        G = np.identity(n)
        G[0][2] = -v/w * np.cos(theta) + v/w * np.cos(theta + w * dt)
        G[1][2] = -v/w * np.sin(theta) + v/w * np.sin(theta + w * dt)
        G[2][2] = 1
        self.sigma = G @ self.sigma @ (G.T) + Res 

if __name__ == '__main__':
    pass 
    n = 1 # number of landmarks 

    # inicializa o estado inicial com n*2 landmark e pose (3)
    x0 = np.zeros(n*2 + 3)
    x0[0] = 0
    x0[1] = 0
    x0[2] = 0

    # Inicializar as incertezas 
    infinito = 10000
    sigma0 = np.identity(n*2 + 3) * infinito
    sigma0[0][0] = 0
    sigma0[1][1] = 0
    sigma0[2][2] = 0

    Q = np.identity(2) # uncertanty in the measurement, bearing and range 
    Q[0][0] = 1
    Q[1][1] = 1

    u = [1,1]
    dt = 1

    np.set_printoptions(suppress=True, formatter={'float_kind':'{:16.3f}'.format}, linewidth=100)
    teste = modelo(x0, dt, sigma0, True)
    # print(teste.x)     
    # print(teste.sigma)
    teste.move(u)
    # print(teste.x)
    # print(teste.sigma)
    teste_ekf = EKF()
    obs = np.array([[2, 2]])
    teste_ekf.correct_prediction(Q, teste, obs, [0])







