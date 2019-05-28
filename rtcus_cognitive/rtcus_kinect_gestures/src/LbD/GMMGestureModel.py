from stats_utils import *
import numpy
import sys
from scipy.cluster.vq import *
import LbD
from LbD.GestureModelBase import *

class GMMGestureModel(GestureModelBase):

    def __init__(self,gesture_name,dimensionality,data_points,stop_threshold,cluster_lenght,meta_tags=[]):
        assert dimensionality == len(data_points)
        self.reset(gesture_name,dimensionality,data_points,stop_threshold,cluster_lenght,meta_tags=meta_tags)

    def reset(self,gesture_name,dimensionality,data_points,stop_threshold,cluster_lenght,meta_tags=[]):
        GestureModelBase.__init__(self,gesture_name,dimensionality,meta_tags=meta_tags)
        assert dimensionality == len(data_points)
        self.D=dimensionality
        self.K=cluster_lenght
        self.__gesture_name=gesture_name
        
        # Covariance matrices (KxDxD)
        self.sigmas = [[] for _ in xrange(self.K)]
        
        # Center matrices (KxD)
        self.mus = [[] for _ in xrange(self.K)]
        
        # Cluster matrices (K x D x p); p = number of points within cluster k
        self.__clusters_data_points = None 
        
        # True if EM algorithm has to keep updating the i-th cluster
        self.update_clusters = [True for _ in xrange(self.K)]
        
        # Priori probabilities (Kx1)
        self.priors = []
        
        # Coordinates of the datapoints to be used in the GMM (DxN)
        self.coordinates= data_points
        self.N=len(data_points[0])
        
        # posterior probabilities of the datapoints (KxN)
        self.gamma = [[0.0 for _ in xrange(self.N)] for _ in xrange(self.K)]
        
        self._initialize_clusters()
        
        self._initialize_stats_parameters()
        
        self._loglikelihood_current=  log_likelihood(self.N,self.K,self.D,self.priors,self.mus,self.sigmas,self.coordinates)
        self.stop_th=stop_threshold
    
    @property
    def gesture_name(self):
        return self.__gesture_name
    
    @staticmethod    
    def create_and_learn(gesture_name,dimensionality,data_points,stop_threshold, max_clusters, cluster_lenght=None):
        if cluster_lenght != None:
            system=GMMGestureModel(gesture_name,dimensionality,data_points,stop_threshold,cluster_lenght)
        else:
            k_opt = 1
            bic_min = sys.maxint
            for num_k in xrange(1,max_clusters+1):
                if num_k == 1:
                    system=GMMGestureModel(gesture_name,dimensionality,data_points,stop_threshold,num_k)
                else:
                    system.reset(gesture_name,dimensionality,data_points,stop_threshold, num_k)
                
                system.learn()
                #print("Likelihood = " + str(-1*system.get_training_set_likelihood()))
                bic = system.get_bic()
                if bic < bic_min:
                    bic_min = bic
                    k_opt = num_k
            system.reset(gesture_name,dimensionality,data_points,stop_threshold, k_opt)

        return system

    # Returns the Bayesian Information Criterion (BIC)    
    def get_bic(self):
        l  = -1*self.get_training_set_likelihood()
        np = (self.K -1) + self.K*(self.D + 0.5*self.D*(self.D + 1))
        return l + 0.5*np*numpy.log10(self.N)

    def get_gaussians(self):
        """mus format : K x matrix(D x 1)"""
        return (self.mus,self.sigmas,self.priors)
        

    def get_training_set_data(self):
        return self.coordinates
    
    def _initialize_stats_parameters(self):
       
        # prior probabilites P(k) in accordance with number of points of the cluster
        for i in xrange(self.K):
            self.priors.append(float(len(self.clusters_data_points[i][0]))/self.N)
                
        # center and covariance matrices:
        for i in xrange(self.K):
            mu = [[] for _ in xrange(self.D)]
            for dim in xrange(self.D):
                mu[dim].append(numpy.mean(self.clusters_data_points[i][dim]))
                
            self.mus[i] = numpy.matrix(mu)
        
            cov_matrix = numpy.cov(self.clusters_data_points[i])
            cov_matrix = numpy.matrix(cov_matrix)
            self.sigmas[i] = cov_matrix

    @property
    def clusters_data_points(self):
        if self.__clusters_data_points==None:
            # First we transpose the coordinates matrix, to get points in rows instead of columns (as required by kmeans2 external method)
            points = numpy.matrix(self.coordinates)
            points = points.transpose()    
        
            self.__clusters_data_points= [[[] for _ in xrange(self.D)] for _ in xrange(self.K)]
            centers, idx = kmeans2(points,self.K,iter=1000)
            print "kmeans executed (req k=%d), number of clusters: %d"%(self.K,len(centers))
            print centers
                
            # Points sorting into clusters_data_points
            for i in xrange(self.N):
                for dim in xrange(self.D):
                    self.__clusters_data_points[idx[i]][dim].append(self.coordinates[dim][i])
                    
        return self.__clusters_data_points
    
    def _initialize_clusters(self):
        # Initialization of GMM parameters
        # ------------------------------------------------
        
        # Initial clustering by k-means segmentation
        empty_cluster = True
        cluster_tries = 0
        

        while empty_cluster and self.K>=1:
            cluster_sizes = []
            print "K size: %d"%self.K
            for current_cluster in xrange(self.K):
                size_k = len(self.clusters_data_points[current_cluster][0])
                cluster_sizes.append(size_k)
                if size_k == 0:
                    print "Empty cluster found. Retrying..."
                    self.__clusters_data_points=None
                    cluster_tries += 1
                    if cluster_tries > 5:
                        print "Impossible to make " + str(self.K) + " clusters_data_points. Trying with " + str(self.K-1) + " clusters_data_points."
                        cluster_tries = 0
                        self.K -= 1
                    break
                elif current_cluster == self.K-1:
                    #print "Finished clustering."
                    #for i in xrange(self.K):
                        #print "Cluster " + str(i+1) + ": " + str(cluster_sizes[i]) + " points."
                    empty_cluster = False

    
    
    def _e_step(self):
        for num_k in xrange(self.K):
            for i in xrange(self.N):
                self.gamma[num_k][i] = pkj(getPoint(self.coordinates, i), num_k, self.D, self.K, self.priors, self.mus, self.sigmas)
            
        eks = []    
        for num_k in xrange(self.K):
            eks.append(Ek(num_k, self.gamma))
             
        return (eks,self.gamma)
    
    def _m_step(self,eks,gamma):
        (K,D,N)=(self.K,self.D,self.N)
        for num_k in xrange(K):
            ek = eks[num_k]
            self.priors[num_k] = pi_k_learning(ek, N)
            self._check_gaussian_update()
            if self.update_clusters[num_k]:
                mu_temp = mu_k_learning(num_k, ek, gamma, N, D, self.coordinates)
                sigma_temp = sigma_k_learning(num_k, ek, gamma, N, D, mu_temp, self.coordinates)
                self.mus[num_k] = mu_temp
                self.sigmas[num_k] = sigma_temp
    
 
    def _check_gaussian_update(self):
        '''
        this method checks if each gaussian component has to be updated. This means that this code affects to the learning loop
        stop condition.
        '''

    def stop_condition(self):
        if self._loglikelihood_previous==-1:
            return False 
        else:
            return (self.get_last_iteration_gain() <self.stop_th) or sum(self.update_clusters) == 0
    
    def learn_step(self):        
        (eks,gamma)=self._e_step()
        self._m_step(eks,gamma)
        return self.compute_gesture_loglikelihood(self.coordinates)
    
    def compute_gesture_likelihood(self,training_data):
        return numpy.exp(self.compute_gesture_loglikelihood(training_data))
        
    def compute_gesture_loglikelihood(self,training_data):
        assert self.D==len(training_data)
        return log_likelihood(len(training_data[0]),self.K,self.D,self.priors,self.mus,self.sigmas,training_data)
    
    def save_to_file(self,path):
        #to avoid big file results
        self.__clusters_data_points=None
        GestureModelBase.save_to_file(self,path)
        
    
