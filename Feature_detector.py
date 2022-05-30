import time
from bagpy import bagreader
import cv2 as cv
import pandas as pd
import numpy as np

def load_bag(name):
    b = bagreader(name)

    # get the list of topics
    table_info = b.topic_table
    
    LASER_MSG = b.message_by_topic('/scan')
    df_laser = pd.read_csv(LASER_MSG)
        
    return df_laser

def laser_data_extraction(df,idx):
    # get distances and angles of a measure
    distance_array = np.array(df.filter(like='range', axis=1).drop(['range_min','range_max'],axis=1))
    distance_array=distance_array[idx,:]

    #get angles
    min_angle = df['angle_min'].loc[df.index[idx]]
    max_angle = df['angle_max'].loc[df.index[idx]]
    increment_angle = df['angle_increment'].loc[df.index[idx]] 

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

def create_map(x,y,res_map):
    #create_map
    map_x = np.arange(0,np.max(x),res_map)
    map_y = np.arange(0,np.max(y),res_map)
    map = np.zeros((map_x.shape[0],map_y.shape[0]))
    idx = [np.abs(np.subtract(np.reshape(x, (-1, 1)), np.reshape(map_x, (1, -1)))).argmin(axis=1),
        np.abs(np.subtract(np.reshape(y, (-1, 1)), np.reshape(map_y, (1, -1)))).argmin(axis=1)]
    map[idx] = 1
    return map

def detect_lines(map,acc_th,min_line_lenght,max_line_gap,plot = False):
    dst = np.array(map*255).astype('uint8')
    element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2, 2))
    dst = cv.dilate(dst, element)
    linesP = cv.HoughLinesP(dst, 1, np.pi / 180,acc_th, None, min_line_lenght, max_line_gap)
    if plot:
        cv.namedWindow('Source', cv.WINDOW_KEEPRATIO)
        cv.namedWindow('Detected Lines (in red) - Probabilistic Line Transform', cv.WINDOW_KEEPRATIO)
        cdstP = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv.line(cdstP, (l[0], l[1]), (l[2], l[3]),
                        (0, 0, 255), 1, cv.LINE_AA)

        cv.imshow("Source", dst)
        cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

        cv.waitKey()
    linesP = np.reshape(linesP,(linesP.shape[0],-1))
    v = np.array([linesP[:, 2]-linesP[:,  0], linesP[:, 3]-linesP[:, 1]]).T
    v = np.divide(v, np.reshape(np.linalg.norm(v, axis=1), (v.shape[0], -1)))
    phis = np.deg2rad(90)-np.arcsin(v[:,1])
    n = np.array([np.cos(phis),np.sin(phis)]).T
    p = np.array(linesP[:,0:2])
    dist = np.abs(np.sum(np.multiply(n,p),axis = 1))
    df = pd.DataFrame(linesP, columns=['x_1', 'y_1', 'x_2', 'y_2'])
    df['rs_line'] = dist
    df['phis_line'] = phis
    return df


def check_points_in_line(x, y, df, min_dist_th):
    points = np.vstack((x*100, y*100)).T
    phis = np.array(df['phis_line'])
    rs = np.array(df['rs_line'])
    npoints = np.zeros(phis.shape)
    error_mse = np.zeros(phis.shape)
    for idx, phi in enumerate(phis):
        b = rs[idx]/np.sin(phis[idx])
        p = np.array([[0, b]]).T
        v = np.array([[-np.sin(phis[idx]), np.cos(phis[idx])]]).T

        dist = np.linalg.norm(np.reshape(np.cross((points-p.T), v.T), (-1, 1)), axis=1)
        keep_idx = np.where(np.abs(dist) < min_dist_th)[0]
        npoints[idx] = keep_idx.shape[0]
        if npoints[idx] != 0:
            error_mse[idx] = (dist[keep_idx]**2).mean()
    df['npoints'] = npoints
    df['error_mse'] = error_mse
    return df


def filter_segments(df, threshold_error,threshold_line):
    threshold_npoints = np.max(df['npoints'])*0.5
    df = df[df['error_mse'] <= threshold_error]
    df = df[df['npoints'] >= threshold_npoints]
    df = df.sort_values(['phis_line','rs_line'])
    df_diff = df[['phis_line', 'rs_line']].diff(periods=1)
    df_diff = df_diff.rename(columns={'phis_line': 'phis_line_diff', 'rs_line': 'rs_line_diff'})
    pd.concat([df, df_diff], axis=1)
    return df
df_laser = load_bag('./23-05/2022-05-23-15-50-47.bag')

rho, theta = laser_data_extraction(df_laser,100)
start_time = time.time()
zS = polar2z( rho, theta)
x = np.real(zS)
y = np.imag(zS)
nx = np.min(x)
ny = np.min(y)
#translate to first quadrant
x = x-nx
y = y-ny
map = create_map(x,y,0.01)
df = detect_lines(map,20,30,30,plot = False)
print("--- %s seconds ---" % (time.time() - start_time))
df = check_points_in_line(x,y,df,20)
print("--- %s seconds ---" % (time.time() - start_time))
df_filtered = filter_segments(df,5,10)

