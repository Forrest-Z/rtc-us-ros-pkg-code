import os
import sys
import yaml
import numpy
import dtw_utils

class GestureEntry():
    """@summary: stores data of a gesture. Can be seen as a set of signals evolving along the time. Several demonstrations can be stored for learning proposes.
    It also have signal accomodation capabilities for a right posteriory learning process"""
    def __init__(self,name,frames,sampling_frequency,time_duration=None,demonstrations=1,demonstrations_names=None):
        nF = len(frames)
        if nF < 1:
            print "Number of relative frames must be > 0!"
            raise Exception("Empty frames list is not a valid arguments")
    
        for f in frames:
            assert all([k in ['position','rotation','fixed','target',"time_variable"] for k in f.keys()])
        
        if demonstrations_names != None:
            assert len(demonstrations_names)==demonstrations,"demonstration count must be coherent with parameters"
            
        self.__name=name
        self.__meta_sampling_frequency=sampling_frequency
        self.__meta_demonstrations_count=demonstrations
        self.__meta_time_duration= time_duration
        self.__meta_initial_param_frames=frames
        #optionally specified
        self.__meta_demonstration_names=demonstrations_names
        
        self.__meta_max_start_time_for_interpolation= None
        self.__meta_min_end_time_for_interpolation= None
        self.__use_time_variable=False
        

        self.__meta_temporal_info=None
        
        #list: demo -> min_frame_demo_count
        self.__meta_min_count_per_demo= None

        self.fixed_frames= [f['fixed'] for f in frames]
        self.target_frames=[f['target'] for f in frames]
        
        self.position_variables=["" for f in frames]
        self.rotation_variables=["" for f in frames]
            
        for i in xrange(len(frames)):
            if frames[i].has_key("position"):
                self.position_variables[i]=frames[i]["position"]
            if frames[i].has_key("rotation"):
                self.rotation_variables[i]=frames[i]["rotation"]        
        
            if self.rotation_variables[i]=="" and self.position_variables[i]=="":
                raise Exception("At least: position or orientation must be specified for each tracking frame")
        
        self.__meta_num_vars= sum([ len(pv) for pv in self.position_variables])+ sum([ len(pv) for pv in self.rotation_variables])
        #demo x frame x samples(time|variables)
        self.raw_data=[[[] for _ in xrange(len(frames))] for _ in xrange(demonstrations)]
        self.__state='sampling_adquisition'
    
    @property
    def name(self):
        return self.__name
    
    def __get_frame_variables(self,findex):
        if 'x' in self.position_variables[findex]:
            yield 'x'
        if 'y' in self.position_variables[findex]:
            yield 'y'            
        if 'z' in self.position_variables[findex]:
            yield 'z'
        if 'r' in self.rotation_variables[findex]:
            yield 'rr'
        if 'p' in self.rotation_variables[findex]:
            yield 'rp'
        if 'y' in self.rotation_variables[findex]:
            yield 'ry'
    
    def __preload_meta(self):
        assert self.demonstration_count == len(self.raw_data), "incoherence between the stored data and the desired number of demonstration"
        assert all([len(self.raw_data[demo_index][0])>0 for demo_index in xrange(self.demonstration_count)]), "all frame of all demonstration have to be filled"
        
        self.__meta_temporal_info=[[] for demo in self.raw_data]
        demo_durations=[]
        for demo_index, demo in enumerate(self.raw_data):
                frame_durations=[]
                for frame_index,samples in enumerate(demo):
                    #print self.demonstration_count
                    #print str(self.demonstrations_names[demo_index])+" || samples: "+str(len(samples))+" ||frame->"+ str(frame_index)
                    start_time=samples[0][0]
                    end_time=samples[-1][0]
                    time_duration= end_time-start_time
                    max_sampling_distance= max ([samples[i+1][0]-samples[i][0] for i in xrange(len(samples)-1)])
                    info={"start_time":start_time,
                          "end_time":end_time,
                          "time_duration":time_duration,
                          "max_sampling_distance":max_sampling_distance,
                          "sampling_period":1.0/self.__meta_sampling_frequency,
                          "sampling_count":len(samples)}
                    self.__meta_temporal_info[demo_index].append(info)
                    
                demo_durations.append(frame_durations)
                
                
        self.__meta_min_end_time_for_interpolation=min([min([self.__meta_temporal_info[demo_index][frame_index]["end_time"]
                                        for frame_index in xrange(len(demo))])
                                        for (demo_index,demo) in enumerate(self.raw_data)])
        
        self.__meta_max_start_time_for_interpolation=max([max([self.__meta_temporal_info[demo_index][frame_index]["start_time"]
                                        for frame_index in xrange(len(demo))])
                                        for (demo_index,demo) in enumerate(self.raw_data)])
        

        #list: demo -> min_frame_demo_count
        self.__meta_min_count_per_demo = [min([len(frame_samples[0]) for frame_samples in demo]) for demo in self.raw_data]
        
        #coherence checks
        assert len(self.raw_data)==self.__meta_demonstrations_count
        assert all([ self.__meta_temporal_info[demo_index][frame_index]["max_sampling_distance"]<self.__meta_sampling_frequency for demo_index,frame_index, _ in self.__all_frames()])
                            
    @property
    def time_duration(self):
        """
        returns the minimal time duration of any recorded demo for any frame
        """
        self.__preload_meta()
        return self.__meta_time_duration

    @property
    def demonstration_count(self):
        return self.__meta_demonstrations_count
    
    @property
    def demonstrations_names(self):
        return self.__meta_demonstration_names
    
    @demonstrations_names.setter
    def demonstrations_names(self,value):
        assert self.__state=="fully_defined"
        assert len(value)==self.demonstration_count
        self.__meta_demonstration_names = value
    
    def increment_demo_count(self):
        self.raw_data.append([[] for _ in xrange(self.frame_count)])
        self.__meta_demonstrations_count+=1
        
    @property
    def use_time_variable(self):
        return self.__use_time_variable
    
    @use_time_variable.setter
    def use_time_variable(self,value):
        self.__use_time_variable=value
    
    @property
    def dimensionality(self):
        if not self.use_time_variable:
            return self.__total_variable_count()
        else:
            return self.__total_variable_count()+1
    
    def __total_variable_count(self):
        return self.__meta_num_vars
    
    def variable_count(self,frame_index):
        """
        summary: use gesture.dimensionality for the total variable count
        """
        return len(self.raw_data[0][frame_index][0])-1
    
    @property
    def frame_count(self):
        return len(self.fixed_frames)

    def get_frame_config(self,index):
        return (self.fixed_frames[index],self.target_frames[index],[ v for v in self.__get_frame_variables(index)])
    
    def get_fixed_frame(self,index):
        return self.fixed_frames[index]
    
    def get_target_frame(self,index):
        return self.target_frames[index]
    
    def __normalize_angle(self,angle,last_angle):
        pi=numpy.pi
        
        """
        #avoid the effect gimbal_lock
        distance= angle-last_angle
        if abs(distance)>2.0*pi:
            if distance > 0.0:
                angle= angle + pi
            else:
                angle = angle -pi
        """     
        #normalize 
        #if angle< -pi:
        #    angle= angle + pi
        #elif angle > pi:
        #    angle = angle - pi
            
        return angle
    
    def push_frame_data_from_tf(self,frame_index,trans,rpy,demonstration,time):
        assert self.__state=="sampling_adquisition"
        
        frame_samples=self.raw_data[demonstration][frame_index]
        
        #check demo time coherence
        if len(frame_samples)>0:
            last_sample=frame_samples[-1]
            assert last_sample[0]<time
        else:
            last_sample= [0.0]*7
            
        values=[]
        values.append(time)
        
        #easily optimizable, soo much "in" operations
        if 'x' in self.position_variables[frame_index]:
            values.append(trans[0])
            print "pushing x in %d:%f"%(frame_index,trans[0])
        if 'y' in self.position_variables[frame_index]:
            values.append(trans[1])
            print "pushing y in %d:%f"%(frame_index,trans[1])
        if 'z' in self.position_variables[frame_index]:
            values.append(trans[2])
            print "pushing z in %d:%f"%(frame_index,trans[2])
        if 'r' in self.rotation_variables[frame_index]:
            values.append(self.__normalize_angle(rpy[0],last_sample[len(values)-1]))
        if 'p' in self.rotation_variables[frame_index]:
            values.append(self.__normalize_angle(rpy[1],last_sample[len(values)-1]))
        if 'y' in self.rotation_variables[frame_index]:
            values.append(self.__normalize_angle(rpy[2],last_sample[len(values)-1]))
            
        frame_samples.append(values)
    
    #FIXME: Is this necessary, coherence checks are being checked very often
    def mark_gesture_defined(self):
        """
        @summary: this method must be called when the process of samples and demonstration adquisition
        has finished 
        """
        
        #more assertions are possible for variable coherence
        self.__state="fully_defined"
        self.__preload_meta()
    
    def get_temporal_info(self,frame_index,demo_index):
        """
        @return: a map with the temporal information given a demonstration and a frame.
        @attention: the format of the map is
        {"start_time":start_time,"end_time":end_time,"time_duration":time_duration,"max_sampling_distance":max_sampling_distance,"sampling_period":1.0/self.__meta_sampling_frequency}
        """
        self.__preload_meta()
        return self.__meta_temporal_info[demo_index][frame_index]
            
                            
    def save_to_file(self,path):
        """
        @summary: stores the gesture in a yaml file in the specified path
        """
        self.__check_and_trunkate_number_of_samples()
        f=open(path, "w")
        yaml.dump(self,f)
        f.close()
        
    def get_6D_frames_from_values(self,findex,values):
        """
        @return a list of variables captured for a frame.
        Possible values are ["x","y","z","rr","rp","ry"]
        """

        vars=["x","y","z","rr","rp","ry"]
        frames=[]
        for findex in xrange(self.frame_count):
            (fixed_frame,target_frame,frame_variables) = self.get_frame_config(findex)
            frame_values=[]
            vindex=0
            for v in vars:
                if v in frame_variables:
                    frame_values.append(values[vindex])
                else:
                    frame_values.append(0)
                vindex=vindex+1
            frames.append(frame_values)
        return frames
    
    def __all_frames(self):
        for demo_index,demo in enumerate(self.raw_data):            
            for frame_index,frame_samples in enumerate(demo):
                yield (demo_index,frame_index,frame_samples)
    
    def __check_and_trunkate_number_of_samples(self):
        """
        to generate a training data table all frames of the same demonstration have to have the same number of samples
        @warning: this is not signal acomodation but a rough method for continue the program
        """
        self.__preload_meta()
        total_min = min (self.__meta_min_count_per_demo)
    
        for demo in self.raw_data:            
            for frame_samples in demo:
                while(len(frame_samples)<total_min):
                    frame_samples.pop()
                         
    def needs_accommodation(self,demo_index=-1):
        """
        @return: a bolean specifing if a sampling accommodation should be done for the specified demonstration. 
        A frame accommodation must be done if not all the frames have the same number of samples
        """
        if demo_index!=-1:
            return not all([ len(frame)==len(self.raw_data[demo_index][0]) for frame in self.raw_data[demo_index]])
        else:
            return not all([ all([len(frame_samples)==len(demo[0]) for frame_samples in demo]) for demo in self.raw_data])
        
    def get_reference_demo(self):
        """
        @return: the demo index that can be considered the most typical in least squares terms/dtw.
        """
        assert not self.needs_accommodation()
        (demo_model_index,distance,distance_maps)=dtw_utils.get_best_dwt_model([numpy.transpose(self.get_training_data(demo_index=demo_a)) for demo_a in xrange(self.demonstration_count)])
        return demo_model_index
    
    def get_best_demos(self,demo_count):
        """
        @return: get the firsts demo_count indexes of demos nearest in terms of least squares/dtw of the best model. This can be
        useful for representation or for discarding outliers  
        """
        assert not self.needs_accommodation()
        assert demo_count <=self.demonstration_count,"the number of requested demos must be lower than %d"%self.demonstration_count
        best_demo_index=self.get_reference_demo()
        best_demo_sequence=numpy.transpose(self.get_training_data(demo_index=best_demo_index))
        distances={}
        
        for demo_index in [i for i in xrange(self.demonstration_count) if i!=best_demo_index]:
            y_sequence=numpy.transpose(self.get_training_data(demo_index=demo_index))
            dist= dtw_utils.dtw_compute_asymetric_distance(best_demo_sequence,y_sequence,self.__used_sakoe_chibah_window(len(best_demo_sequence),len(y_sequence)))
            distances[demo_index]=dist
        
        ordered_distances=sorted(distances.items(), key=lambda x: x[1])
        return [di for di,dist in ordered_distances[0:demo_count]]
        
    
    def __used_sakoe_chibah_window(self,len_x_sequence,len_y_sequence):
        percent=numpy.double(0.5)
        return dtw_utils.dtw_sakoe_chiba_window(len_x_sequence, len_y_sequence,percent*min(self.__meta_min_count_per_demo))
        
    def process_signal_accommodation(self,offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True,time_wrapping=False):
        """
        @return:  the time variable after the signal accomodation
        @param offset_accomodate: shift the first sample of each demo to the start time = 0
        @param demo_longitude_accomodate: get the min demo duration and disccard/cut all the demos to fit with that shortest demo 
        """
        self.__preload_meta()
        if offset_accomodate:
            #each demonstration go to zero time (one frame will have a t=0.0 sample)
            earlier_start_time_per_demo = [(demo_index,min([self.get_temporal_info(frame_index, demo_index)["start_time"] for (frame_index,_) in enumerate(demo)])) for  (demo_index,demo) in enumerate(self.raw_data)]
            for demo_index,min_start_time in earlier_start_time_per_demo:
                for frame_samples in self.raw_data[demo_index]:
                    for sample in frame_samples:
                        sample[0]=sample[0]-min_start_time
                        
        self.__preload_meta()                
        if demo_longitude_accomodate:
            assert offset_accomodate
            earlier_end_time = min([min([self.get_temporal_info(frame_index, demo_index)["end_time"] for (frame_index,_) in enumerate(demo)]) for  (demo_index,demo) in enumerate(self.raw_data)])
            #pay attention: duration modified
            for demo_index,demo in enumerate(self.raw_data):
                for frame_index,frame_samples in enumerate(demo):
                    try:
                        fist_out_index = [sample[0]>earlier_end_time for sample in frame_samples].index(True)
                        self.raw_data[demo_index][frame_index]=frame_samples[0:fist_out_index]    
                    except:
                        continue
            #regular_sampling_time=arange(0,,self.__meta_time_duration)
        self.__preload_meta()                
        
        regular_sampling_time= numpy.arange(self.__meta_max_start_time_for_interpolation,self.__meta_min_end_time_for_interpolation,step=self.__meta_sampling_frequency**(-1.0))
        
        if regular_sampling_acomodation:
            import scipy.interpolate
            for demo_index,demo in enumerate(self.raw_data):
                for frame_index,frame_samples in enumerate(demo):
                    (fixed_frame,target_frame,variables)=self.get_frame_config(frame_index)
                    samples=numpy.matrix(frame_samples)
                    time= [ t[0,0] for t in samples[:,0]]
                    new_data=[]
                    new_data.append(regular_sampling_time)
                    for var_index,var in enumerate(variables):
                        ordenades=[val[0,0] for val in samples[:,var_index+1]]
                        f = scipy.interpolate.interp1d(time, ordenades)
                        new_data.append(f(regular_sampling_time))
                    
                    self.raw_data[demo_index][frame_index]=numpy.transpose(new_data)
                    
            self.__preload_meta()                
            earlier_start_time_per_demo = [(demo_index,min([self.get_temporal_info(frame_index, demo_index)["start_time"] for (frame_index,_) in enumerate(demo)])) for  (demo_index,demo) in enumerate(self.raw_data)]
            for demo_index,min_start_time in earlier_start_time_per_demo:
                for frame_samples in self.raw_data[demo_index]:
                    for sample in frame_samples:
                        sample[0]=sample[0]-min_start_time
            self.__preload_meta()
            
            if time_wrapping==True:
                assert regular_sampling_acomodation==True
                assert not self.needs_accommodation()
                
                #GET TIME WRAPPINGS FOR REST OF DEMOS
                (demo_model_index,distance,distance_maps)=dtw_utils.get_best_dwt_model([numpy.transpose(self.get_training_data(demo_index=demo_a)) for demo_a in xrange(self.demonstration_count)])
                
                #print "others distances:"+str(distances)
                reference_demo=self.get_training_data(demo_index=demo_model_index)
                regular_sampling_time= numpy.arange(self.__meta_max_start_time_for_interpolation,self.__meta_min_end_time_for_interpolation,step=self.__meta_sampling_frequency**(-1.0))
                
                for demo_index in xrange(self.demonstration_count):
                    if demo_index!=demo_model_index:
                        (distance_map,n,m)=distance_maps[(demo_model_index,demo_index)]
                        (pathx,pathy)=dtw_utils.dtw_optimal_warping_path(distance_map, n, m, True)
                     
                        #READJUSTING DEMO DATA
                        i=0
                        new_demo_data=[[] for _ in xrange(self.frame_count)]
                        while i<len(pathx) and i < len(regular_sampling_time):
                            tx_index=pathx[i]
                            #print (tx_index, len(regular_sampling_time),pathx[-1])
                            x_time= regular_sampling_time[tx_index]
                            entries=[]
                            
                            #ellapsed y-samples in the tx_index instant
                            while i< len(pathx) and pathx[i]==tx_index:
                                ty_index=pathy[i]
                                entries.append(ty_index)
                                i+=1
                            
                            #y signal samples in the same x_time
                            if len(entries)>=2:
                                #time y-samples
                                time_samples= [regular_sampling_time[ty] for ty in entries]
                                #central time
                                middle_time= (time_samples[-1] + time_samples[0])/2.0
                                
                                for frame_index in xrange(self.frame_count):
                                    new_frame_entry=[]
                                    #put the time
                                    new_frame_entry.append(x_time)
                                    
                                    for var_index in xrange(self.variable_count(frame_index)):
                                        ordenades = [self.raw_data[demo_index][frame_index][ty][var_index+1] for ty in entries]
                                        f = scipy.interpolate.interp1d(time_samples, ordenades)
                                        new_frame_entry.append(f(middle_time))
                                    new_demo_data[frame_index].append(new_frame_entry)
                                
                            #y signal expansion -> to the same value, same throttle    
                            elif len(entries)==1:
                                for frame_index in xrange(self.frame_count):
                                    new_frame_entry=[]
                                    new_frame_entry.append(x_time)
                                    ty_index=entries[0]
                                    for var_index in xrange(self.variable_count(frame_index)):
                                        new_frame_entry.append(self.raw_data[demo_index][frame_index][ty_index][var_index+1])
                                    new_demo_data[frame_index].append(new_frame_entry)
                            else:
                                raise Exception("Not Implemented")
                            
                            
                        self.raw_data[demo_index]=new_demo_data        
                        
            self.__preload_meta()
                
                
            return regular_sampling_time
    
    def __get_variable_colum_index(self,frame_index,variable):
        """
        @summary: get the index of a specific variable or of a specific frame
        """
        if frame_index >0:
            frame_offset=sum([self.variable_count(frame_index=fi) for fi in xrange(frame_index)])
        else:
            frame_offset=0
        
        #according to push_frame_tf this is the strict order of variable storage into the raw table
        vars=[ v for v in self.__get_frame_variables(frame_index)]
        var_index=vars.index(variable)
        if self.use_time_variable:
            var_index+=1
            
        #print "frame index offset:%d"%frame_offset
        #print "frame index var %s in:%d"%(variable,frame_offset+var_index)
        return frame_offset+var_index
        
    def get_task_space_matrix_projection(self,frame_index,full_covariance_matrix):
        """
        @summary: given a covariance matrix computed from the training data with dimensions
        len(variables) x len(variables), get the 3D projection of the x,y,z variables
        of the frame frame_index
        """
        frame_variables=self.__get_frame_variables(frame_index)
        assert "x"in frame_variables and "y" in frame_variables and "z" in frame_variables
        
        x_index=self.__get_variable_colum_index(frame_index,"x")
        #get the positional covariance 3D projection
        return full_covariance_matrix[x_index:x_index+3,x_index:x_index+3]
        
        
    def get_training_data(self,demo_index=None,frame_index=None,scale=1.0):
        """@return: a matrix (2dlist) variables x sample_value (without the time column)
           @warning: this format is not the same than get_frame_training_data_and_time
           @attention: time is not included in the matrix if it has not been specifically catched
           and demonstrations are concatenated (independently of the time of each sample of each demonstration)
           @see: process_signal_accommodation
        """
        #should assert the time coherence?
        assert not self.needs_accommodation()
        
        if(self.use_time_variable):
            time_offset=0
        else:
            time_offset=1
        
        if frame_index==None:    
            frame_range= xrange(self.frame_count)
            table_data=[[] for _ in xrange(self.dimensionality )]
        else:
            assert frame_index >=0 and frame_index<self.frame_count
            frame_range=[frame_index]
            table_data= [[] for _ in xrange(self.variable_count(frame_index)+(1-time_offset))]
        
        if demo_index==None:
            demo_range=xrange(self.demonstration_count)
        else:
            assert demo_index >=0 and demo_index<self.demonstration_count
            demo_range=[demo_index]
        
        for di in demo_range:
            if(self.use_time_variable):
                time_offset=0
            else:
                time_offset=1
                
            demo=self.raw_data[di]
            var_offset=0
            for fi in frame_range:
                samples=demo[fi]
                for var_index in xrange(self.variable_count(fi)+(1-time_offset)):
                    for sample in samples:
                        value=sample[var_index+time_offset]*scale
                        table_data[var_offset+var_index].append(value)
                        
                var_offset=var_offset+ self.variable_count(fi)+(1-time_offset)
                #just the first frame take the time variable
                if time_offset==0:
                    time_offset=1
        
        assert all([len(var_samples)>0 for var_samples in table_data])
        return table_data
                        
    def get_merged_training_data(self,scale=1.0):
        """
        @return: a matrix (2D list) (time|variables) x samples (time if use_time==True)
        @attention: time is not included in the matrix if it has not been specifically catched
         and demonstrations are concatenated (independently of the time of each sample of each demonstration)
        @see: process_signal_accommodation
        """
        assert self.demonstration_count>0
        assert not self.needs_accommodation()
        
        #remove this and use more severe assertions?
        self.__check_and_trunkate_number_of_samples()
        table_data= self.get_training_data(demo_index=0)
        for demo_index in xrange(0,self.demonstration_count-1):
            frame_data = self.get_training_data(demo_index=demo_index,scale=scale)
            for var_index in xrange(self.__meta_num_vars):
                table_data[var_index]=table_data[var_index]+frame_data[var_index]
            
        return table_data
    
    def split(self,gesture_entry_names,gesture_count=None):
        """
        @summary: mainly for training datadatabase handling. Ie: get some positive examples as the training set and the others
        for model evaluation.
        @param gesture_count:can be a list to specify the number of demonstration for each demo, for instance split(3,[3,4,3]) divides the
        gesture in three gestures with 3,4 and 3 demonstrations, precondition: sum(gesture_count)==demonstration_count 
        """
        count = len(gesture_entry_names)
        if(gesture_count!=None):
            assert sum(gesture_count)==self.demonstration_count, "the gesture should be splitted properly in a meaningful number of demnostrations"
        else:
            gesture_count=[ int(self.demonstration_count/count) for x in xrange(count)]
            if self.demonstration_count%count!=0:
                gesture_count[-1]+=self.demonstration_count%count
        
        #print "gesture groups %s"%str(gesture_count)
        demonstration_offset=0
        gestures=[]
        
        this_demo_indexes= [i for i in xrange(self.demonstration_count)]
        #print "this demo indexes %s"%(str(this_demo_indexes))
        import random
        random.shuffle(this_demo_indexes)
        shuffled_indexes=this_demo_indexes
        #print "shufled demoindexes %s"%(str(shuffled_indexes))
        indexes_offset=0
        for (derived_gesture_index,dcount) in enumerate(gesture_count):
            #dth represent one of the subgestures with dcount demos
            
            dth_gesture_indexes= shuffled_indexes[indexes_offset:indexes_offset+dcount]
            indexes_offset+=dcount
            if self.demonstrations_names!=None:
                sub_demonstration_names=[self.demonstrations_names[index] for index in dth_gesture_indexes]
                #sub_demonstration_names=sub_demonstration_names+self.demonstrations_names[demonstration_offset:demonstration_offset+dcount]
            else:
                sub_demonstration_names=None
                
            #print "derived gesture: %d - demoindexes: %s -subdemonstrations names: %s"%(derived_gesture_index,str(dth_gesture_indexes),str(sub_demonstration_names))
            #same gesture name but different gestureentryname
            g=GestureEntry(self.name,self.__meta_initial_param_frames,self.__meta_sampling_frequency,self.time_duration,dcount,sub_demonstration_names)
            
            assert g.demonstration_count==dcount, "post condition of coherence"
            for i,demo_index in enumerate(dth_gesture_indexes):
                for frame_index, frame_samples in enumerate(self.raw_data[demo_index]):
                    g.raw_data[i][frame_index]= numpy.copy(frame_samples)
            
            demonstration_offset+=dcount    
            g.mark_gesture_defined()
            gestures.append(g)
            
        return gestures

    @staticmethod
    def load_from_file(path):
        f=open(path)
        gesture=yaml.load(f)
        if not isinstance(gesture,GestureEntry):
            raise Exception("Invalid file data")
        return gesture
