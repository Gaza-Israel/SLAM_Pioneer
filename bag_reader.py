import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import cos,sin
from skimage.feature import peak_local_max
from scipy import ndimage, misc
import matplotlib.pyplot

def load_bag(name):
    b = bagreader(name)

    # get the list of topics
    table_info = b.topic_table
    
    LASER_MSG = b.message_by_topic('/scan')
    df_laser = pd.read_csv(LASER_MSG)
        
    return df_laser

def feature_extraction(df):
    # get distances and angles of a measure
    distance_array = np.array(df.filter(like='range', axis=1).drop(['range_min','range_max'],axis=1))
    distance_array=distance_array[0,:]
    
    #get angles
    min_angle = df['angle_min'].loc[df.index[0]]
    max_angle = df['angle_max'].loc[df.index[0]]
    increment_angle = df['angle_increment'].loc[df.index[0]] 

    angle_array = np.arange(min_angle, max_angle+increment_angle ,increment_angle)

    array_has_nan = np.logical_or(np.isnan(distance_array) , distance_array<0.05)
    distance_array = distance_array[np.invert(array_has_nan)]
    angle_array = angle_array[np.invert(array_has_nan)]
    
    return (distance_array,angle_array)

def polar2z(r,theta):
    return r * np.exp( 1j * theta )

def z2polar(x,y):
    z = x + 1j * y
    return ( np.abs(z), np.angle(z) )

def polar2cart(d,theta): #inputs as array
    # convert polar coordinate to cartesian coordinate
    x = np.multiply(d,np.cos(theta))
    y = np.multiply(d,np.sin(theta))
    return (x,y)


class feature_detector:
    def __init__(self,phi_resolution = 0.1,r_resolution = 0.01,convolution_filter_size = 5,min_th = 20,threshold_error = 0.01,threshold_npoints = 20):
        self.phi_resolution =  phi_resolution
        self.r_resolution = r_resolution 
        self.Max_dist = 0
        self.min_th = min_th
        self.convolution_filter_size = convolution_filter_size
        self.threshold_error = threshold_error
        self.threshold_npoints = threshold_npoints
        
    def hough_transform(self,x,y):
        Nx = np.max(x)-np.min(x)
        Ny = np.max(y)-np.min(y)
        self.Max_dist = np.sqrt(Nx**2 + Ny ** 2)
        phis = np.deg2rad(np.arange(-90, 90,self.phi_resolution))
        rs = np.arange(-self.Max_dist,self.Max_dist,self.r_resolution)
        accumulator = np.zeros((rs.shape[0],phis.shape[0]))
        for point in zip(x, y):
            for phi_idx,phi in enumerate(phis):
                r = point[0]*cos(phi) + point[1]*sin(phi)
                r_idx = (np.abs(r - rs)).argmin()
                accumulator[r_idx,phi_idx] += 1
        filtered_accumulator = ndimage.uniform_filter(accumulator, size=self.convolution_filter_size)*self.convolution_filter_size
        return filtered_accumulator, phis, rs
    def detect_peaks(self,accumulator):            
        acc_peaks = peak_local_max(accumulator, min_distance=35,threshold_abs=self.min_th,threshold_rel=0.15, exclude_border=False)
        return acc_peaks
    
    def extract_segments(self,x,y,rs,phis,th):
        points = np.vstack((x,y)).T
        endpoints = np.zeros((phis.shape[0],4))
        npoints = np.zeros(phis.shape)
        error_mse = np.zeros(phis.shape)
        for idx,phi in enumerate(phis):
            b = rs[idx]/np.sin(phis[idx])
            p = np.array([[0,b]]).T
            v = np.array([[-np.sin(phis[idx]),np.cos(phis[idx])]]).T
              
            
            
            dist = np.linalg.norm(np.reshape(np.cross((points-p.T),v.T), (-1, 1)),axis = 1)
            keep_idx = np.where(np.abs(dist)<th)[0]
            npoints[idx]=keep_idx.shape[0]
            
            
            i = np.array([[1,0],[0,1]])
            projection_matrix = np.dot(i.T,v)/np.dot(v.T,v)*v.T
            p_projected = p-np.dot(projection_matrix,p)
            P = np.hstack((projection_matrix,p_projected))
            points_projected = np.dot(P,np.hstack((points[keep_idx],np.ones((points[keep_idx].shape[0],1)))).T).T
            error_mse[idx] = (dist[keep_idx]**2).mean()
            if phis[idx]==np.deg2rad(90):
                max_projected = np.argmax(points_projected[:,1],axis = 0)
                min_projected = np.argmin(points_projected[:,1],axis = 0)
                endpoints[idx,:] = np.hstack((points_projected[max_projected,:],points_projected[min_projected,:]))
            else:
                max_projected = np.argmax(points_projected[:,0],axis = 0)
                min_projected = np.argmin(points_projected[:,0],axis = 0)
                endpoints[idx,:] = np.hstack((points_projected[max_projected,:],points_projected[min_projected,:]))
        
        df = pd.DataFrame(endpoints,columns=['x_1','y_1','x_2','y_2'])
        df['rs_line'] = rs
        df['phis_line'] = phis
        df['npoints'] = npoints
        df['error_mse'] = error_mse
        
        return endpoints,df
    def filter_segments(self,df):
        df = df[df['error_mse']<=self.threshold_error]
        df = df[df['npoints']>=self.threshold_npoints]
        return df
    def find_intersections(self):
        a = 1
    def extract_features(self):
        a = 1
    def plot_hough_space(self,x,y):
        accumulator, phis, rs = self.hough_transform(x,y)
        # plt.figure('Hough Space')
        plt.imshow(accumulator)
        plt.set_cmap('gray')
        plt.show()
    def plot_lines(self,x,y,df=None):
        
        plt.figure()
        plt.scatter(x,y,color = 'k',linewidths=0.5)
        plt.plot([df['x_1'],df['x_2']],[df['y_1'],df['y_2']])


df_laser = load_bag('./23-05/2022-05-23-15-50-47.bag')

rho,theta = feature_extraction(df_laser)


zS = polar2z( rho, theta)
x = np.real(zS)
y = np.imag(zS)
# x = np.hstack((x,np.array([1,2,3,4,5,6])))
# y = np.hstack((y,np.array([1,2,3,4,5,6])))
nx = x-np.min(x)
ny = y-np.min(y)
# nx = np.array([0,1,2,3,0,1,2,3])
# ny = np.array([0,3,6,9,0,2,4,6])


ft = feature_detector()

ft.plot_hough_space(nx, ny)

accumulator, phis, rs = ft.hough_transform(nx,ny)
acc_peaks = ft.detect_peaks(accumulator)
endpoints, df_lines_info = ft.extract_segments(nx, ny, rs[acc_peaks[:,0]],phis[acc_peaks[:,1]],0.3)

ft.plot_lines(nx, ny,df_lines_info)


df_lines_info_segments = ft.filter_segments(df_lines_info)


ft.plot_lines(nx, ny,df_lines_info_segments)


