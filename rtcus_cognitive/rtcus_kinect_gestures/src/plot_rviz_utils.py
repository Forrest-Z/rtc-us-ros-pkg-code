from colorsys import hsv_to_rgb        
import os
import numpy
import rospy
import PyKDL
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def getRGB_color_from_index(i,total):
        (K,num_k)=(total,i)
        h_val = num_k/float(K)
        (r,g,b) = hsv_to_rgb(h_val,1.0,1.0)
        return (r,g,b)
    
# Autovector creation
def markerVector(frame_id,id_m,vector,position=None,color=(0.5,0.5,0.5),namespace="default_ns"):
    if position==None:
        position=numpy.zeros((3,1))
        
    marker = Marker ()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now ()
    marker.id = id_m
    marker.ns=namespace
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x=0.004
    marker.scale.y=0.008
    marker.scale.z=0.004
    marker.color.a= 0.5
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    (start,end)=(Point(),Point())
    start.x = position[0,0]
    start.y = position[1,0]
    start.z = position[2,0]
    end.x=start.x+vector[0,0]
    end.y=start.y+vector[0,1]
    end.z=start.z+vector[0,2]
    marker.points.append(start)
    marker.points.append(end)
    return marker

def plot_points_set_rviz(gesture,clusters,startid):
    # Clusters datapoints markers plotting
    m_id = startid
    ms=[]
    K=len(clusters)
    for id_cluster in xrange(K):
        curr_cluster=clusters[id_cluster]
        (r,g,b)=getRGB_color_from_index(id_cluster,K)
        for i in xrange(len(curr_cluster[0])):
            for fi in xrange(gesture.frame_count):
                (fixed_frame,target_frame,variables)=gesture.get_frame_config(fi)
                marker = Marker ()
                marker.header.frame_id = fixed_frame
                marker.ns="points"
                marker.header.stamp = rospy.Time.now ()
                marker.id = m_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(curr_cluster[fi*3][i])
                marker.pose.position.y = float(curr_cluster[fi*3+1][i])
                marker.pose.position.z = float(curr_cluster[fi*3+2][i])
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1
                #prob = float(point_GMM_prob(numpy.matrix([[curr_cluster[0][i]],[curr_cluster[1][i]],[curr_cluster[2][i]]]), D,K,k,mus,sigmas)*10000)
                marker.scale.x = 0.01 #prob
                marker.scale.y = 0.01 #prob
                marker.scale.z = 0.01 #prob
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a=1.0
                m_id += 1
                # Publish the Marker
                ms.append(marker)
    return (m_id,ms)

def plot_gaussians_rviz(gesture,mus,sigmas,startid,gaussian_marker_pub):
    elip_id = startid
    markers=[]
    K=len(mus)
    
    for num_k in xrange(K):
        #for eeach frame a 3D projection of the ND cluster 
        for fi in xrange(gesture.frame_count):
            # Gaussian ellipsoid marker
            
            (fixed_frame,target_frame,variables)=gesture.get_frame_config(fi)
            marker = Marker ()
            marker.header.frame_id =fixed_frame
            marker.ns="gaussian"
            marker.header.stamp = rospy.Time.now ()
            marker.id = elip_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = mus[num_k][fi*3+0]
            marker.pose.position.y = mus[num_k][fi*3+1]
            marker.pose.position.z = mus[num_k][fi*3+2]
            (r,g,b)=getRGB_color_from_index(num_k,K)
            marker.color.a = 0.3
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            #print "K = " + str(num_k) + ". Color = (" + str(r) + "," + str(g) + "," + str(b) + ")"
    
            #print "total matrix:"+str(sigmas[num_k])
            projected_frame_sigma= gesture.get_task_space_matrix_projection(fi,sigmas[num_k])
            #print "projected fi:%d :"%fi+str(projected_frame_sigma)
            # Distribution eigen vectors and values
            (eigValues,eigVectors) = numpy.linalg.eig(projected_frame_sigma)
            eigVectors=eigVectors.transpose()
            
            # Eigen vectors plotting
            vec_color=[(1.0,0,0),(0,1.0,0),(0,0,1.0)]
            id_m=0
            for v in eigVectors:
                elip_id += 1
                #print elip_id
                m=markerVector(fixed_frame,elip_id, v*eigValues[id_m]/2.0, position=mus[num_k][fi*3:fi*3+3,0],color= vec_color[id_m],namespace="autovectors")
                id_m += 1
                gaussian_marker_pub.publish(m)
            
            
            
            # Rotation markers
            eigx_n=-PyKDL.Vector(eigVectors[0,0],eigVectors[0,1],eigVectors[0,2])
            eigy_n=-PyKDL.Vector(eigVectors[1,0],eigVectors[1,1],eigVectors[1,2])
            eigz_n=-PyKDL.Vector(eigVectors[2,0],eigVectors[2,1],eigVectors[2,2])
            rot = PyKDL.Rotation (eigx_n,eigy_n,eigz_n)
    
            quat = rot.GetQuaternion ()
            rpy = rot.GetRPY()
            #quat=(quat[0]/quat[3],quat[1]/quat[3],quat[2]/quat[3],quat[3]/quat[3])
    
            rot_quat=PyKDL.Rotation.Quaternion(quat[0],quat[1],quat[2],quat[3])
        
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
    
    
            marker.scale.x = eigValues[0]*100
            marker.scale.y = eigValues[1]*100
            marker.scale.z = eigValues[2]*100
            elip_id += 1
            markers.append(marker)
            
    for m in markers:
        gaussian_marker_pub.publish(m)
    
    return (elip_id,markers)


def gaussian_2d_ellipse_from_covariance_matrix(covariance_matrix, mu_vector, project_variables=None):
    assert isinstance(covariance_matrix, numpy.matrix)
    assert project_variables==None or isinstance(project_variables,list)
    
    if project_variables!=None:
        assert len(project_variables)==2 and all([var>=0 and var<len(covariance_matrix) for var in project_variables])
        _i0=project_variables[0]
        _i1=project_variables[1]
        covariance_matrix=numpy.matrix([[covariance_matrix[_i0,_i0],covariance_matrix[_i0,_i1]],[covariance_matrix[_i1,_i0],covariance_matrix[_i1,_i1]]])
        
    
    (eigValues,eigVectors) = numpy.linalg.eig(covariance_matrix)
    angle = numpy.arctan2(eigVectors[1,0],eigVectors[0,0])*180.0/numpy.pi
    scale=eigValues
    position= mu_vector
    
    return (angle,scale,position)