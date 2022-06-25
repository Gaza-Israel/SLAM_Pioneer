from ctypes import sizeof
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

class EKF():
    def __init__(self):
        self.H = 0 # Jacobian for the observation
        self.z_predict = 0 # Predicted location of the given landmark 
        self.K = 0 # Kalmann Gain
        self.conta_landmarks = 0 
        self.r_measure = 0
        self.phi_measure = 0

    def correct_prediction(self, Q, modelo, landmarks, idx_landmarks):
        k = 0
        for i in landmarks:
            j = idx_landmarks[k] 
            self.obs_predict(j, landmarks[k], modelo)
            self.KalmanGain(modelo, Q)
            land_range_bearing = np.array([self.r_measure, self.phi_measure])
            # print("Rm", land_range_bearing[0], "Tm", np.rad2deg(land_range_bearing[1]))
            modelo.x_estimate = modelo.x_estimate + self.K @ (land_range_bearing - self.z_predict)
            # print("Re", self.z_predict[0], "Te", np.rad2deg(self.z_predict[1]))
            # print((land_range_bearing[0] - self.z_predict[0]), self.K[0])
            # print("Raio = ", (self.z_predict[0]), modelo.x[0])
            # print("Phi = ", (np.rad2deg(self.z_predict[1])))

            # print("Raio = ", (self.z_predict[0] - land_range_bearing[0]), "Kalman = ", self.K[0])
            # print("Phi = ", (np.rad2deg(self.z_predict[1]- land_range_bearing[1])), "Kalman = ", self.K[1])

            # print((np.rad2deg(land_range_bearing[1]) - np.rad2deg(self.z_predict[1])), self.K[1])
            # print(self.K @ (land_range_bearing - self.z_predict))
            # print(((land_range_bearing - self.z_predict)))
            # print("Ganho Kalman", self.K )
            # print("Re", self.z_predict[0], "Te", np.rad2deg(self.z_predict[1])))
            # print("Dif medicao", (land_range_bearixng - self.z_predict))
            # print("Raio medido = ", self.r, "Raio robo = ", self.z_predict[0])
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
        # print(x,y,theta, modelo.x[0], modelo.x[1], modelo.x[2])
        
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
        # print("Xr ", x, "Lx", land_x, "Yr", y, "Ly", land_y)
        # print("R", self.r_measure, "Phi", np.rad2deg(self.phi_measure))

        pos_robo = np.array([x,y])
        # print("pos", pos_robo[0], pos_land[0])
        delta = np.array(pos_land - pos_robo)
        # print("delrta", delta[0], delta[1])
        q = delta.T @ delta
        # print(pos_robo[0], pos_land[0])
        self.z_predict = np.array([np.sqrt(q), np.arctan2(delta[1], delta[0]) - theta]) # prevendo a posicao da landmark  
        # print("raio = ", self.z_predict[0], "theta = ", np.rad2deg(self.z_predict[1]), np.rad2deg(theta))

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
    def __init__(self, x0, dt, sigma0, teste): # x = [x, y, theta], u = [velocity, angular_velocity] 
        self.x = x0 # State vector for the model, [x_coord, y_coord, theta, land_1_x, land_1_y, land_2_x, land_2_y ...]
        self.x_estimate = x0
        self.dt = dt # Time step 
        self.sigma = sigma0 # Covariances matrix 
        self.sigma_estimate = sigma0
        self.teste = teste
        

    def move(self, u): # updates de pose and covariance of the robot for a given control input 
        n = len(self.x_estimate)
        F = np.zeros((3,n))
        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1

        dist  = u[0]*self.dt
        dth = u[1]*self.dt
        dx = F.T @ np.array([np.cos(self.x[2]+ dth)*dist, np.sin(self.x[2]+ dth)*dist, dth])
        self.x_estimate = np.add(self.x, dx)
        # print(self.x, "\n", dx,  "\n",self.x_estimate)
        
        if self.teste:
            self.x = self.x_estimate

        self.update_cov(u) 

    
 
    def update_cov(self, u): # updates de covariance matrix for a given input
        # This changes the uncertainty in the robot pose but not in the landmar position
        v = u[0]
        w = u[1]
        theta = self.x_estimate[2]
        dt = self.dt
        n = len(self.x_estimate)

        Res = np.zeros((n,n)) # FALTA COMPLETAR - User defined uncertainty in the range and bearing of the model 
        Res[0][0] = 1e-4
        Res[1][1] = 1e-4
        Res[2][2] = np.deg2rad(1)

        G = np.identity(n)
        G[0][2] = -np.sin(w * dt + theta) * (v * dt)
        G[1][2] = np.cos(w * dt + theta) * (v * dt)
        self.sigma = G @ self.sigma @ (G.T) + Res 

if __name__ == '__main__':

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

    u = [1,0]
    dt = 1

    np.set_printoptions(suppress=True, formatter={'float_kind':'{:16.3f}'.format}, linewidth=10)
    teste = modelo(x0, dt, sigma0)
    teste_ekf = EKF()
    obs = np.array([[2, 2]])
    teste_ekf.correct_prediction(Q, teste, obs, [0])

    # print(np.arctan2(1,1), np.tan(np.arctan2(1,1)))
    # print(np.arctan2(1,0), np.tan(np.arctan2(1,0)))
    # print(np.arctan2(0,1), np.tan(np.arctan2(0,1)))






