from stats_utils import *
import numpy
from scipy.cluster.vq import *
import LbD
from LbD.LbDSystem import *

class rtcusLbDSystem(LbDSystem):
    def _initialize_stats_parameters(self):
         LbDSystem._initialize_stats_parameters(self)
         print "original mu:"+str(self.mus)
         for i in xrange(len(self.mus)):
             self.mus[i]=numpy.multiply(1.1,self.mus[i])
 