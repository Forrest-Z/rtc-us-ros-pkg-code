import numpy

def get_best_dwt_model(demos,constr=None):
    """
    @summary: by default the sakoe_chiba_windows used is a 10% of the number
    of samples
    """
        
    distances=[numpy.double() for _ in xrange(len(demos))]
    distance_maps={}
    for demo_a in xrange(len(demos)):
        for demo_b in xrange(demo_a+1,len(demos)):
            x=demos[demo_a]
            y=demos[demo_b]
            constr=dtw_sakoe_chiba_window(len(x), len(y),0.1*numpy.min([len(x),len(y)]))
            (distance,distance_map)=dtw_compute_asymmetric_with_map(x, y, constr)
            distance_maps[(demo_a,demo_b)]=(distance_map,len(x),len(y))
            distance_maps[(demo_b,demo_a)]=(numpy.transpose(distance_map),len(y),len(x))
            #print "distance (%d,%d)=%s"%(demo_a,demo_b,str(distance))
            #print distance_map[0:1][0:4]
            distances[demo_a]+= distance**2
            distances[demo_b]+= distance**2
    
    best_demo_index=numpy.argmin(distances)
    best_demo_expected_distance=numpy.sqrt(distances[best_demo_index])/numpy.double(len(demos))
    #print "BEST DEMO: "+str(best_demo_expected_distance)+ " -  "+str(best_demo_index)
    return best_demo_index,best_demo_expected_distance,distance_maps

def __euclidean(a,b):
    return numpy.linalg.norm(numpy.array(a)-numpy.array(b))

def dtw_compute_asymetric_distance(x,y,__constr):
    """
    @summary: calculate the dtw asymetric distance
    @attention: matrix of dimensions vars x samples
    """
    dist = 0.0
    d=0.0

    n = len(x)
    m = len(y)
    
    dtwm_i=numpy.double()
    dtwm_j=numpy.double()
    dtwm_ij=numpy.double()
  
    prev_col =  [0.0 for _ in xrange(m)]
    next_col = [0.0 for _ in xrange(m)]
    
    dbl_info=numpy.finfo(numpy.float64)
    for j in xrange(0,m):
        prev_col[j] = dbl_info.max
        next_col[j] = dbl_info.max  

    prev_col[0] = __euclidean(x[0], y[0]); # DP-algorithm (i - 1), (j - 1) 
    
    for j in xrange(__constr[0][0] + 1,__constr[1][0]+1):
        d = __euclidean(x[0], y[j]);
        dtwm_j  = prev_col[j - 1]; # DP-algorithm (j - 1)
        prev_col[j] = dtwm_j;
    
    for i in xrange(1,n):
        for j in xrange(__constr[0][i], __constr[1][i]+1):
            d = __euclidean(x[i], y[j]); 
            if j == 0:
                dtwm_i = prev_col[j] + d; # DP-algorithm (i - 1)
                next_col[j] = dtwm_i;
            else:
                dtwm_i  = prev_col[j];
                dtwm_ij = prev_col[j - 1];
                dtwm_j  = next_col[j - 1];
                
                if dtwm_i != dbl_info.max:
                    dtwm_i = dtwm_i + d; # DP-algorithm (i - 1)
              
                if dtwm_ij != dbl_info.max:
                    dtwm_ij = dtwm_ij + d; # DP-algorithm (i - 1), (j - 1) 
              
                # DP-algorithm (j - 1) (no sum)
                next_col[j] = min(dtwm_i, dtwm_j, dtwm_ij);
            
        dist = next_col[m - 1];
        tmp = prev_col;
        prev_col = next_col;
        next_col = tmp;
        
        for  j in xrange(0, m):
            next_col[j] = dbl_info.max;      
    
    return dist / numpy.double(n) 

def dtw_compute_asymmetric_with_map( x,y,__constr):
    #double *dtwm -> parameter
    n = len(x)
    m = len(y)
    
    dbl_info=numpy.finfo(numpy.float64)
    dtwm=[[dbl_info.max for _ in xrange(m)] for _ in xrange(n)]
    
    d=numpy.double()
    dtwm_i=numpy.double()
    dtwm_j=numpy.double() 
    dtwm_ij=numpy.double()
    
    dtwm[0][0]= __euclidean(x[0], y[0])
    
    for j in xrange(__constr[0][0] + 1,__constr[1][0]+1):
        d = __euclidean(x[0], y[j]);
        dtwm_j  = dtwm[0][j - 1]; # DP-algorithm (j - 1)
        dtwm[0][j] = dtwm_j;
        
    
    for i in xrange(1,n):               
        for j in xrange(__constr[0][i], __constr[1][i]+1):
            d = __euclidean(x[i], y[j]); 
            if j == 0:
                dtwm_i = dtwm[i - 1][j] + d;# DP-algorithm (i - 1)
                dtwm[i][j] = dtwm_i;
            else:
                dtwm_i  = dtwm[i - 1][j];
                dtwm_ij = dtwm[i - 1][j - 1];
                dtwm_j  = dtwm[i][j - 1];
                
                if dtwm_i != dbl_info.max:
                    dtwm_i = dtwm_i + d; # DP-algorithm (i - 1)
              
                if dtwm_ij != dbl_info.max:
                    dtwm_ij = dtwm_ij + d; # DP-algorithm (i - 1), (j - 1) 
              
                # DP-algorithm (j - 1) (no sum)
                dtwm[i][j] = min(dtwm_i, dtwm_j, dtwm_ij);
                

    return (dtwm[-1][- 1] / numpy.double(n),dtwm);

def dtw_optimal_warping_path(dtwm, n, m,  startbc):
    # returns -> int *pathx, int *pathy
    i = n - 1;
    j = m - 1;
    
    min_ij=numpy.double()
    dtwm_i=numpy.double()
    dtwm_j=numpy.double() 
    dtwm_ij=numpy.double()

    pathx=[]
    pathy=[]
    
    pathx.append(i)
    pathy.append(j)

    while (i > 0) or (j > 0):
        if (i == 0) and (j > 0):
            if startbc == 1:
                j -= 1;
            else:
                break;
      
      
        if (j == 0) and (i > 0):
            if startbc == 1:
                i -= 1;
            else:
                break;
      
      
        if (i > 0) and (j > 0):
            dtwm_i  = dtwm[i - 1][j];
            dtwm_j  = dtwm[i][j - 1];
            dtwm_ij = dtwm[i - 1][j - 1];
            min_ij  = min(dtwm_i, dtwm_j, dtwm_ij);
            
            if dtwm_ij == min_ij:
                i -= 1;
                j -= 1;
            elif dtwm_i == min_ij:
                i -= 1;
            elif dtwm_j == min_ij:
                j -= 1;
                
        pathx.append(i);
        pathy.append(j);
    pathx.reverse()
    pathy.reverse()
    return (pathx,pathy)

def dtw_no_window(n,m):
        __constr=[[0 for _ in xrange(0,n)],[0 for _ in xrange(0,n)]]
        for i in xrange(0,n):
            __constr[0][i] = 0;
            __constr[1][i] = m - 1;
        return __constr;
    
def dtw_sakoe_chiba_window(n,m,r):
    __constr=[[0 for _ in xrange(0,n)],[0 for _ in xrange(0,n)]]
    mnf = numpy.double(m) / numpy.double(n);
    for i in xrange(0,n):
        __constr[0][i] = max( int(numpy.ceil(i * mnf - r)), 0 );
        __constr[1][i] = numpy.int(min( numpy.floor(i * mnf + r), m - 1 ));
    return __constr;

