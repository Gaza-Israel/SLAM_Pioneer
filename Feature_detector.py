from math import pi, ceil
import time
from bagpy import bagreader
import cv2 as cv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans, MiniBatchKMeans
import time

def load_bag(name):
    b = bagreader(name)

    # get the list of topics
    table_info = b.topic_table

    LASER_MSG = b.message_by_topic("/scan")
    df_laser = pd.read_csv(LASER_MSG)

    return df_laser


def laser_data_extraction(df, idx):
    # get distances and angles of a measure
    distance_array = np.array(df.filter(like="range", axis=1).drop(["range_min", "range_max"], axis=1))
    distance_array = distance_array[idx, :]

    # get angles
    min_angle = df["angle_min"].loc[df.index[idx]]
    max_angle = df["angle_max"].loc[df.index[idx]]
    increment_angle = df["angle_increment"].loc[df.index[idx]]

    angle_array = np.arange(min_angle, max_angle + increment_angle, increment_angle)

    array_has_nan = np.logical_or(np.isnan(distance_array), distance_array < 0.05)
    distance_array = distance_array[np.invert(array_has_nan)]
    angle_array = angle_array[np.invert(array_has_nan)]

    return (distance_array, angle_array)


def polar2z(r, theta):
    zS = r * np.exp(1j * theta)
    x = np.real(zS)
    y = np.imag(zS)

    return x, y


def z2polar(x, y):
    z = x + 1j * y
    return (np.abs(z), np.angle(z))


class feature_detector:
    def __init__(self, laser_max_range, res_map, acc_th, min_line_lenght, max_line_gap, min_dist2line_th, filter_min_points,threshold_cluster,max_intersection_distance) -> None:
        self.x_min = -laser_max_range - 0.8
        self.y_min = -laser_max_range - 0.8
        self.x_max = laser_max_range + 0.8
        self.y_max = laser_max_range + 0.8
        self.rs_diag = ((self.x_max - self.x_min) ** 2 + (self.y_max - self.y_min) ** 2) ** 0.5
        self.phis_diag = pi
        self.res_map = res_map
        self.acc_th = acc_th
        self.min_line_lenght = np.round(min_line_lenght / self.res_map).astype(int)
        self.max_line_gap = np.round(max_line_gap / self.res_map).astype(int)
        self.min_dist2line_th = np.round(min_dist2line_th / self.res_map).astype(int)
        self.filter_min_points = filter_min_points
        self.threshold_cluster = threshold_cluster
        self.max_intersection_distance = np.round(max_intersection_distance / self.res_map).astype(int)
        pass

    def create_map(self, x, y):
        # create_map
        map_x = np.arange(self.x_min, self.x_max, self.res_map)
        map_y = np.arange(self.y_min, self.y_max, self.res_map)
        map = np.zeros((map_x.shape[0], map_y.shape[0]))
        self.map_x_size = map.shape[0]
        self.map_y_size = map.shape[1]
        idx = (
            np.abs(np.subtract(np.reshape(x, (-1, 1)), np.reshape(map_x, (1, -1)))).argmin(axis=1),
            np.abs(np.subtract(np.reshape(y, (-1, 1)), np.reshape(map_y, (1, -1)))).argmin(axis=1),
        )
        map[idx] = 1
        return map, idx

    def detect_lines(self, map, plot=False):
        
        dst = np.array(map * 255).astype("uint8")
        element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2, 2))
        dst = cv.dilate(dst, element)
        linesP = cv.HoughLinesP(dst, 1, np.pi / 180, self.acc_th, None, self.min_line_lenght, self.max_line_gap)
        linesP = np.reshape(linesP, (linesP.shape[0], -1))
        v = np.array([linesP[:, 2] - linesP[:, 0], linesP[:, 3] - linesP[:, 1]]).T
        v = np.divide(v, np.reshape(np.linalg.norm(v, axis=1), (v.shape[0], -1)))
        phis = np.deg2rad(90) + np.arcsin(v[:, 1])
        dist = linesP[:, 0] * np.cos(phis) + linesP[:, 1] * np.sin(phis)
        
        
        cdstP = None
        if plot:
            cdstP = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
            cv.namedWindow("Source", cv.WINDOW_KEEPRATIO)
            if linesP is not None:
                for i in range(0, len(linesP)):
                    l = linesP[i]
                    cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 4, cv.LINE_AA)

                    phi = phis[i]
                    r = dist[i]
                    a = np.cos(phi)
                    b = np.sin(phi)
                    x0 = a * r
                    y0 = b * r
                    pt1 = (int(x0 + 10000 * (-b)), int(y0 + 10000 * (a)))
                    pt2 = (int(x0 - 10000 * (-b)), int(y0 - 10000 * (a)))
                    cv.line(cdstP, pt1, pt2, (255, 0, 255), 1, cv.LINE_AA)
            cv.circle(
                cdstP, (np.ceil(np.size(map, 0) / 2).astype(int), np.ceil(np.size(map, 0) / 2).astype(int),), 4, (0, 255, 0), -1,
            )
            cv.imshow("Source", cv.rotate(dst, cv.ROTATE_180))
            cv.waitKey(1)
            
        df = pd.DataFrame(linesP, columns=["x_1", "y_1", "x_2", "y_2"])
        df["rs_line"] = dist
        df["phis_line"] = phis
        return df, cdstP

    def check_points_in_line(self, points, df):
        phis = np.array(df["phis_line"])
        rs = np.array(df["rs_line"])
        npoints = np.zeros(phis.shape)
        error_mse = np.zeros(phis.shape)
        for idx, phi in enumerate(phis):
            dist = np.abs(np.sum((np.multiply(points[1], np.cos(phi)), np.multiply(points[0], np.sin(phi))), axis=0,) - rs[idx])
            keep_idx = np.where(dist <= self.min_dist2line_th)[0]
            npoints[idx] = keep_idx.shape[0]
            if npoints[idx] != 0:
                error_mse[idx] = (dist[keep_idx] ** 2).mean()
        df["npoints"] = npoints
        df["error_mse"] = error_mse
        return df

    def filter_segments(self, df, img = None,online=True):
        
        if online:
            df1 = df.copy()
            df1 = df1[df1['npoints']>20]
            df1['mean_error'] = df1['error_mse']/ df1['npoints']
            df1 = df1[df1['mean_error']<2]
            df1 = df1.sort_values(by=["rs_line", "phis_line",'mean_error'])
            
            df1["rs_line_norm"] = df1["rs_line"] / self.rs_diag * self.res_map * 10
            df1["phis_line_norm"] = df1["phis_line"] / self.phis_diag 
            df1['rs_line_diff'] = df1['rs_line_norm'].diff()
            df1['phis_line_diff'] = df1['phis_line_norm'].diff()
            df1['error_measure'] = df1['rs_line_diff']**2 + df1['phis_line_diff']**2
            df1=df1.fillna(1)
            df1 = df1[df1['error_measure']>0.09]

        else:
            start_time = time.time()
            df = df[df["npoints"] > self.filter_min_points]
    
            n_cluster = len(df)
            
            
            x = df[["rs_line", "phis_line"]]
            x["rs_line"] = x["rs_line"] / self.rs_diag * self.res_map
            x["phis_line"] = x["phis_line"] / self.phis_diag 
            # x['x_mean'] = (df['x_1']+df['x_2'])/2 / self.rs_diag * self.res_map 
            # x['y_mean'] = (df['y_1']+df['y_2'])/2 / self.rs_diag * self.res_map 
    
            print("--- %s seconds - FILTER -----  Prep -----" % (time.time() - start_time))
            if len(df) != 0:
                wcss = []   
                for i in range(1, n_cluster):
                    kmeans = MiniBatchKMeans(i)
                    kmeans.fit(x)
                    wcss_iter = kmeans.inertia_
                    wcss.append(wcss_iter)
                    if i>2:
                        if wcss[i-1]-wcss[i-2]<self.threshold_cluster:
                            i+=100
    
                print("--- %s seconds - FILTER -----  fit -----" % (time.time() - start_time))
                wcss_norm = wcss
                wcss_diff = -1 * np.diff(wcss_norm)
                idx = np.argmax(wcss_diff < self.threshold_cluster) + 1
                number_clusters = range(1, n_cluster)
    
                # plt.plot(number_clusters[0:-1], wcss_diff)
                # plt.title("The Elbow title")
                # plt.xlabel("Number of clusters")
                # plt.ylabel("WCSS")
                # plt.show()
    
                # plt.plot(number_clusters[1:], wcss[1:])
                # plt.title("The Elbow title")
                # plt.xlabel("Number of clusters")
                # plt.ylabel("WCSS")
                # plt.show()
    
                print("--- %s seconds - FILTER -----  find -----" % (time.time() - start_time))
                kmeans = MiniBatchKMeans(idx)
                kmeans.fit(x)
    
                identified_clusters = kmeans.fit_predict(x)
                data_with_clusters = df.copy()
                data_with_clusters["Clusters"] = identified_clusters
                
                # plt.scatter(data_with_clusters["rs_line"],data_with_clusters["phis_line"],c=data_with_clusters["Clusters"],cmap="rainbow",)
                # plt.show()
    
                plt.scatter(x["rs_line"], x["phis_line"], c=data_with_clusters["Clusters"], cmap="rainbow")
                plt.xlabel("Normalized distance")
                plt.ylabel("Normalized angle")
                plt.show()
    
                print("--- %s seconds - FILTER -----  predict -----" % (time.time() - start_time))
                df1 = data_with_clusters.groupby(["Clusters"], as_index=False).agg({"error_mse": "min"})
                df1 = data_with_clusters.merge(df1, how="inner", on=["Clusters", "error_mse"])


        if img is not None:
            
            linesP = np.array(df1[["x_1", "y_1", "x_2", "y_2"]])
            phis = np.array(df1[["phis_line"]])
            dist = np.array(df1[["rs_line"]])
            for i in range(0, len(linesP)):
                l = linesP[i]
                cv.line(img, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 4, cv.LINE_AA)
    
                phi = phis[i]
                r = dist[i]
                a = np.cos(phi)
                b = np.sin(phi)
                x0 = a * r
                y0 = b * r
                pt1 = (int(x0 + 10000 * (-b)), int(y0 + 10000 * (a)))
                pt2 = (int(x0 - 10000 * (-b)), int(y0 - 10000 * (a)))
                cv.line(img, pt1, pt2, (255, 0, 255), 1, cv.LINE_AA)
    
            cv.circle(
                img,
                (
                    np.ceil(np.size(map, 0) / 2).astype(int),
                    np.ceil(np.size(map, 0) / 2).astype(int),
                ),
                4,
                (0, 255, 0),
                -1,
            )
            cv.waitKey(1)
        return df1, img

    def find_intersections(self, df, img=None, window="Detected Lines (in red) - Probabilistic Line Transform"):
        # x = (r*sin(phi1)-r1*sin(phi))/sin(phi1-phi)
        # y = (r1*cos(phi)-r*cos(phi1))/sin(phi1-phi)
        x_inter = []
        y_inter = []
        phi1 = np.array(df["phis_line"])
        r1 = np.array(df["rs_line"])
        for idx in range(df.shape[0] - 1):
            phi = np.array(df.iloc[[idx]]["phis_line"])
            r = np.array(df.iloc[[idx]]["rs_line"])
            with np.errstate(divide="ignore", invalid="ignore"):
                x_inter.extend((r * np.sin(phi1[idx + 1 :]) - r1[idx + 1 :] * np.sin(phi)) / np.sin(phi1[idx + 1 :] - phi))
                y_inter.extend((r1[idx + 1 :] * np.cos(phi) - r * np.cos(phi1[idx + 1 :])) / np.sin(phi1[idx + 1 :] - phi))
            np.seterr(divide="warn", invalid="warn")
        center = (self.map_x_size / 2, self.map_y_size / 2)
        dist = np.sqrt(np.power(np.subtract(x_inter, center[0]), 2) + np.power(np.subtract(y_inter, center[1]), 2))
        intersections_df = pd.DataFrame(x_inter, columns=["x"])
        intersections_df["y"] = y_inter
        intersections_df["dist"] = dist
        intersections_df = intersections_df.dropna()
        intersections_df.drop_duplicates()
        intersections_df = intersections_df[intersections_df["dist"] < self.max_intersection_distance]
        intersections_df = intersections_df.reset_index()
        if img is not None:
            for index, row in intersections_df.iterrows():
                cv.rectangle(
                    img, (np.floor(row["x"] - 3).astype("uint16"), np.floor(row["y"] + 3).astype("uint16"),), (np.floor(row["x"] + 3).astype("uint16"), np.floor(row["y"] - 3).astype("uint16"),), (0, 255, 255), 1,
                )
            cv.circle(
                img, (np.ceil(np.size(map, 0) / 2).astype(int), np.ceil(np.size(map, 0) / 2).astype(int),), self.max_intersection_distance, (0, 255, 50), 1,
            )
            cv.imshow(
                window, cv.rotate(img, cv.ROTATE_180),
            )
            cv.waitKey(1)
        return intersections_df,img

    def inter2feature(self, df_inter):
        df_inter_copy = df_inter.copy()
        df_inter_copy['x'] = self.x_min + df_inter['x']*(self.x_max-self.x_min)/self.map_x_size
        df_inter_copy['y'] = self.y_min + df_inter['y']*(self.y_max-self.y_min)/self.map_y_size
        df_inter_copy['dist'] = df_inter['dist']*(self.y_max-self.y_min)/self.map_y_size
        return df_inter_copy

class feature_matcher:
    def __init__(self,dist_th) -> None:
        self.dist_th = dist_th
        pass
    def match_features(self,current_features,map_features,n_map_features,robot_position,img = None,features_px = None):
        current_features_arr = np.reshape(np.array([[current_features['x']],[current_features['y']]]).T,(-1,2))
        
        x = robot_position[0]
        y = robot_position[1]
        k = 0
        for i in current_features:
            x_land = current_features_arr[k][0] 
            y_land = current_features_arr[k][1] 
            land_pos = np.array([x_land, y_land])

            theta = robot_position[2]   
            c = np.cos(theta) # checar se graus ou rad 
            s = np.sin(theta)
            R_frame = np.array([[c, -s], [s, c]])

            land_pos_global = R_frame @ land_pos # coordenadas da landmark no sistema de ref global 
            current_features[k][0] = land_pos_global[1]
            current_features[k][1] = land_pos_global[0]
            k = k + 1

        #FIX features reference frame transform (account for robot heading)
        current_features_w = np.subtract(current_features_arr,(robot_position[0],robot_position[1]))
        #-------------------------------------------------------------------
        idxs = []
        new_features = 0
        if img is not None:
            img_r = cv.rotate(img, cv.ROTATE_180)
        if not map_features.shape[0]==0:
            for idx,feature in enumerate(current_features_w):
                dist = np.sqrt(np.power(np.subtract(feature[0],map_features[:,0]),2) + np.power(np.subtract(feature[1],map_features[:,1]),2))
                min_dist_idx = dist.argmin()
                if dist[min_dist_idx]<self.dist_th and min_dist_idx<n_map_features:
                    idxs.append(min_dist_idx)
                else:
                    idxs.append(n_map_features+new_features)
                    new_features += 1
                if img is not None:
                    center = np.round(img.shape[0]/2).astype(int)
                    point = [np.round(-((features_px['x'][idx]).astype("int")-center))+center + 5,(-(np.round(features_px['y'][idx]).astype("int")-center))+center+0]
                    cv.putText(img_r,str(idxs[-1]),point, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv.LINE_AA)
        else:
            for idx,feature in enumerate(current_features_w):
                idxs.append(n_map_features+new_features)
                new_features += 1
                if img is not None:
                    center = np.round(img.shape[0]/2).astype(int)
                    point = [np.round(-((features_px['x'][idx]).astype("int")-center))+center + 5,(-(np.round(features_px['y'][idx]).astype("int")-center))+center+0]
                    cv.putText(img_r,str(idxs[-1]),point, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv.LINE_AA)
        if img is not None:
            img = cv.rotate(img_r, cv.ROTATE_180)
            cv.imshow("Detected Lines (in red) - Probabilistic Line Transform",cv.rotate(img, cv.ROTATE_180))
            cv.waitKey(1)

        return idxs,new_features
        
if __name__=="__main__":
    df_laser = load_bag("2022-05-23-15-50-47.bag")
    fd = feature_detector(laser_max_range=5.6, res_map=0.01, acc_th=20, min_line_lenght=0.30, max_line_gap=0.30, min_dist2line_th=0.2, filter_min_points = 20,threshold_cluster=0.02,max_intersection_distance=8)
    fm = feature_matcher(0.2)
    map_features = np.zeros((30,2))
    n_map_features = 0
    for idx in range(1550, 2000):
        rho, theta = laser_data_extraction(df_laser, idx)
        start_time = time.time()
        x, y = polar2z(rho, theta)

        map, map_points = fd.create_map(x, y)
        df, img = fd.detect_lines(map, plot=True)
        print("--- %s seconds - Detect Lines ---" % (time.time() - start_time))
        
        df = fd.check_points_in_line(map_points, df)
        print("--- %s seconds - Check points in line ---" % (time.time() - start_time))
        
        # df_inter_not_filtered = fd.find_intersections(df, img, window="Not Filtered") ##DESCOMENTE AQUI PARA O RELATORIO
        dst = np.array(map * 255).astype("uint8")
        cdstP = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
        df_filtered, img2 = fd.filter_segments(df, cdstP)
        print("--- %s seconds - FILTER---" % (time.time() - start_time))
        df_inter_filtered, img = fd.find_intersections(df_filtered, img, window="NOTFiltered_1")

        df_inter_filtered, img = fd.find_intersections(df_filtered, img2, window="Filtered_2")

        print("--- %s seconds - Intersections---" % (time.time() - start_time))
        features = fd.inter2feature(df_inter_filtered)

        idx_feature,new_features = fm.match_features(features,map_features,n_map_features,(0,0)#,img,df_inter_filtered# 
        )
        
        print("--- %s seconds - End loop---" % (time.time() - start_time))
        n_map_features += new_features
        map_features[idx_feature,:] = np.reshape(np.array([[features['x']],[features['y']]]).T,(-1,2))


        print("--------------------------------")
        print("                               ")
        print("--------------------------------")
        
        # time.sleep(1)
        # print(idx_feature)

        #############
        # df_filtered,img = fd.filter_segments(df, 5, 10, plot=True)




