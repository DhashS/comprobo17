from loader import robot
import tf

from geometry_msgs.msg import Point32

import numpy as np

from sklearn.cluster import KMeans
from sklearn.metrics import mean_squared_error
from scipy.ndimage import affine_transform
from scipy.optimize import minimize

class Follower(robot.Neato):
    def __init__(self, ip, rate=5):
        super(Follower, self).__init__(ip, rate)
        self.centroid = None
        self.point_dim = 3 #we live in 3-space
        self.motion_tol = 10 #tuned experimentally, what amount of motion (L_2 norm) between ICP'd clouds
                             #constitutes a thing moving.
                             #may have to be tuned realtime

    def logic(self):
       #let's think about all clouds in /odom
       
       scan_before_last = self.tf.transformPointCloud('/odom', self.historical_clouds[-1])
       last_scan = self.tf.transformPointCloud('/odom', self.last_scan)
       
       #do the aligned scans have an l2 cost of more than the robot's tolerance to motion
       trans_rot = self.ICP(scan_before_last, last_scan)
       if self.motion_tol > self.pcl_tformed_cost(pcl2, pcl1, trans_rot):
           if not self.last_movement_centroid:
               self.last_movement_centroid = self.last_pos
           self.pub.publish(self.movement(self.last_movement_centroid))
           return

       #It's all raw pointwise ops now, so reassign
       scan_before_last = scan_before_last.points
       last_scan = last_scan.points

       #solve for the differences on a pointwise base
       #k, v since it's a dict
       deltas = [self.point_norm(self.point_diff(x, y)) for x, y in self.greed_points.items()]
       
       #K-means cluster them into points that move because feet, and points that move because drift
       kmeans_mask = [bool(x) for x in KMeans(n_clusters=2).fit(np.atleast_2d(deltas).T).labels_]

       #isolate the two clusters, which is motion and which is not?
       #the one with a higher delta avg is the cluster that's moving
       drift_1 = np.mean(deltas[kmeans_mask])
       drift_2 = np.mean(deltas[np.logical_not(kmeans_mask)])
       if drift_1 > drift_2:
           feet_points = last_scan[kmeans_mask]
       else:
           feet_points = last_scan[np.logical_not(kmeans_mask)]
       
       #calculate centroid of feet and move towards it
       self.last_movement_centroid = self.point_avg(feet_points)
       self.pub.publish(self.movement(self.last_movement_centroid))
       
    def apply_tr(self, pcloud, (trans, rot)):
        return affine_transform(self.get_points(pcloud), rot, offset=trans)

    def ICP(self, pcl1, pcl2):
        #let's transform pcl1 to pcl2
        greed = self.greedy_point_select(pcl1, pcl2)
        self.greed_points = greed
        #zero translation for xyz, zero rotation is 
        i_trans, i_rot = ([0]*3), np.eye(3)
        #pack
        initial = np.vstack((i_rot, i_trans))
        new_cloud = pcl1
        #iterativley refine affine transform matricies
        res_mats = []
        while True:
            res = minimize(lambda x: self.pcl_tformed_cost(pcl2, new_cloud, x),
                           initial)
            new_cloud = [Point(x,y,z) for x,y,z in self.apply_tr(pcl1, (res.x[:self.point_dim], res.x[self.point_dim]))]
            if res.x == initial:
                #reached full convergence
                f_trans, f_rot = i_trans, i_rot
                for tr in res_mats:
                    rot = tr[:self.point_dim]
                    trans = tr[self.point_dim]
                    f_trans = f_trans + trans
                    rot = np.dot(f_rot, rot)
                return (f_trans, rot)
            else:
                #update the cloud and the greed dictiopnary
                new_cloud = [Point32(x,y,z) for x,y,z in self.apply_tr(new_cloud, (res.x[:self.point_dim], res.x[self.point_dim]))]
                self.greed_points = self.greedy_point_select(new_cloud, pcl2)
                res_mats.append(res.x)


    def greedy_point_select(self, pcl1, pcl2):
        greed_points = {}
        for pt in pcl1.points:
            #O(n^2)
            #It's the min normed distance point in pc2
            closest_point = pcl2.points[np.argmin([self.point_norm(self.point_diff(x, pt)) for x in pcl2.points])]
            greed_points[pt] = closest_point
        return greed_points


    def pcl_tformed_cost(self, truth, pcl, trans_rot):
        trans, rot = trans_rot[:self.point_dim], trans_rot[self.point_dim]
        pts = self.apply_tr(pcl, (trans, rot))
        truth_pts = self.get_points([self.greed_points[p] for p in [Point32(x,y,z) for x,y,z in pts]])
        accum = 0
        for pt in abs(pts - truth_pts):
            accum += sum(pt**2)
        return accum

        

        
    @staticmethod
    def get_points(pts):
        return np.array([[getattr(pt, attr) for attr in 'xyz'] for pt in pts])

    @staticmethod
    def point_avg(plist):
        collective_point = np.array([0, 0, 0])
        for pt in plist:
            collective_point = collective_point + np.array([getattr(pt, attr) for attr in 'xyz'])
        collective_point = collective_point / len(plist)
        return Point(x=collective_point[0],
                     y=collective_point[1],
                     z=collective_point[2])
    @staticmethod
    #Point32 -> Point32 -> Point32
    def point_diff(this, that):
        return Point32(x=this.x - that.x,
                       y=this.y - that.y,
                       z=this.z - that.z)

    @staticmethod
    #Point32 -> Float
    def point_norm(point):
        pts = [getattr(point, attr) for attr in 'xyz']
        return np.linalg.norm(pts)
    
my_neato = Follower(ip, rate)
my_neato.run()
