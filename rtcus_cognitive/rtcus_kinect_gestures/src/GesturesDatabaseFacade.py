#!/usr/bin/env python
import roslib
from LbD import GMMGestureModel, DTWGestureModel, HMMGestureModel,GestureModelBase
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import os
import LbD
import re
from LbD.GestureEntry import GestureEntry


class DatabaseBase:
    def _get_files_and_names(self,directory,filter):
        file_entries= os.listdir(directory)
        files=[fentry for fentry in file_entries if os.path.isfile(directory+"/"+fentry)]
        file_tuples=[]
        
        for f in files:
            splitted=f.split(".")
            
            if len(splitted)>=2 and all([ filter[-i-1]== splitted[-i-1] for i,filt in enumerate(filter)]):
                full_file_path=directory+"/"+f
                file_name="".join([ s for s in self.__rebuild_name(splitted[:-len(filter)])])
                file_tuples.append((full_file_path,file_name))
        
        return file_tuples
    
    def __rebuild_name(self,strings):
        if strings ==None or len(strings)==0:
            yield ""
        yield strings[0]
        for i in xrange(1,len(strings)):
            yield "."
            yield strings[i]
    
class GestureDatabase(DatabaseBase):
    def __init__(self):
        self.GESTURE_DATABASE_DIRECTORY = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data/gestures'
        self.MODELS_DATABASE_DIRECTORY = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data/models'
        assert os.path.exists(self.GESTURE_DATABASE_DIRECTORY) and not os.path.isfile(self.GESTURE_DATABASE_DIRECTORY)
        
    def get_gesture_entries_names(self):
        return [name for filepath,name in self._get_files_and_names(self.GESTURE_DATABASE_DIRECTORY,["gesture","yaml"]) if os.path.exists(filepath) and not name[0]=="."]
    
    def get_models_names(self,name_regex=None,model_type=None,model_type_name=None):
        filter=["model","yaml"]
        if model_type!=None:
            filter.insert(0, self.__get_model_extesion(model_type))
        elif model_type_name!=None:
            extension=[ self.__get_model_extesion(t) for t in self.__get_model_types() if t.__name__==model_type_name ][0]
            filter.insert(0, extension)
        
        if model_type_name==None:
            models_names=[name for filepath,name in self._get_files_and_names(self.MODELS_DATABASE_DIRECTORY,filter) if os.path.exists(filepath) and not name[0]=="."]
        else:
            models_names=[name+"."+extension for filepath,name in self._get_files_and_names(self.MODELS_DATABASE_DIRECTORY,filter) if os.path.exists(filepath) and not name[0]=="."]
            
        if name_regex!=None:
            models_names= [name for name in self.get_models_names() if re.match(name_regex,name)]
            models_names=set(models_names)
        return models_names
    
    def get_models_by_gesture_name(self,name_regex,model_type=None):
        models_names=self.get_models_names(name_regex)
        return [self.get_model(name,model_type)for name in models_names]
    
    def get_gesture_entry(self,gesture_name):
        return GestureEntry.load_from_file(self.GESTURE_DATABASE_DIRECTORY+"/"+gesture_name+".gesture.yaml")
    
    def save_gesture_entry(self,gesture_name,gesture):
        output_gesture_name=self.GESTURE_DATABASE_DIRECTORY+"/"+gesture_name+".gesture.yaml"
        rospy.loginfo("saving gesture as %s",output_gesture_name)
        gesture.save_to_file(output_gesture_name)
    
    def __get_model_types(self):
        return [LbD.GMMGestureModel.GMMGestureModel,LbD.DTWGestureModel.DTWGestureModel,LbD.HMMGestureModel.HMMGestureModel]
    
    def __get_model_extesion(self,class_type):
        extension=""
        #change this to a abstract property in gesture base
        if class_type == LbD.GMMGestureModel.GMMGestureModel:
            extension="gmm"
        elif class_type== LbD.DTWGestureModel.DTWGestureModel:
            extension="dtw"
        elif  class_type==LbD.HMMGestureModel.HMMGestureModel:
            extension="hmm"
        else:
            raise Exception()
        return extension
    
    def save_model(self,model_name,model):
        extension="."+self.__get_model_extesion(model.__class__)
        filename=self.MODELS_DATABASE_DIRECTORY+"/"+model_name+extension+".model.yaml"
        model.save_to_file(filename)
        
    def get_model(self,model_name, model_type=None):
        try:
            if model_type!= None:
                extension="."+self.__get_model_extesion(model_type)
                filename=self.MODELS_DATABASE_DIRECTORY+"/"+model_name+extension+".model.yaml"
                return GestureModelBase.GestureModelBase.load_from_file(filename)
            else:
                filename=self.MODELS_DATABASE_DIRECTORY+"/"+model_name+".model.yaml"
                return GestureModelBase.GestureModelBase.load_from_file(filename)
        except Exception, e:
            raise Exception(e.message+": filename->"+model_name)
        
    
class BagDatabase(DatabaseBase):
    def __init__(self):
        self.BAGS_DIRECTORY = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data/bags'
        self.OUTPUT_GESTURE_DATABASE_DIRECTORY = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data/gestures/'
        assert os.path.exists(self.BAGS_DIRECTORY) and not os.path.isfile(self.BAGS_DIRECTORY)

    def get_uncategorized_gesture_bag_names(self):
        rospy.loginfo("looking for gestures in bags root")
        return [ filename for fpath, filename in self._get_files_and_names(self.BAGS_DIRECTORY,["bag"]) if not "." in filename]
    
    def get_root_config_files(self):
        rospy.loginfo("looking for config files in: %s",self.BAGS_DIRECTORY)
        return [ (fpath,filename) for fpath, filename in self._get_files_and_names(self.BAGS_DIRECTORY,["config","yaml"]) if not "." in filename] 
    
    def get_gesture_entries_names(self):
        return [fentry for fentry in os.listdir(self.BAGS_DIRECTORY) if not os.path.isfile(self.BAGS_DIRECTORY+"/"+fentry) and not "." in fentry]
    
    def get_gesture_bags(self,gesture_name):
        gesture_resource_path=self.BAGS_DIRECTORY+"/"+gesture_name
        gesture_bag_type=self.__gesture_bag_type(gesture_name)
        #check if directory (cagegorized) or uncategorized bag file
        if "root_uncategorized"==gesture_bag_type:
            #root bag files
            local_bag_files=self._get_files_and_names(self.BAGS_DIRECTORY,["bag"])
            return [(filepath,name) for (filepath,name) in local_bag_files if name==gesture_name]
         
        elif "folder_categorized"==gesture_bag_type:
            bag_directory=gesture_resource_path
            if not (os.path.exists(bag_directory) or os.path.isfile(bag_directory)):
                rospy.logfatal("Incorrect gesture specified: %s",gesture_name)
                raise Exception ("Invalid gesture")
            else:
                local_bag_files=self._get_files_and_names(bag_directory,["bag"])
                if len(local_bag_files)==0:
                    rospy.logwarn("no bag files found for %s. Ignoring folder.",gesture_name)
                
                return local_bag_files 
    
    def __gesture_bag_type(self,gesture_name):
        gesture_resource_path=self.BAGS_DIRECTORY+"/"+gesture_name
        if os.path.exists(gesture_resource_path+".bag") and os.path.isfile(gesture_resource_path+".bag"):
            return "root_uncategorized"
        elif os.path.exists(gesture_resource_path) and not os.path.isfile(gesture_resource_path):
            return "folder_categorized"
        else:
            raise Exception("incorrect gesture")
        
    def get_gesture_config(self,gesture_name):
        """config file must be called equals than the folder"""
        if self.__gesture_bag_type(gesture_name)=="folder_categorized":
            gesture_resource_path=self.BAGS_DIRECTORY+"/"+gesture_name
            config_files=self._get_files_and_names(gesture_resource_path,["config","yaml"])
            if(len(config_files)>0):
                return config_files[0]
            else:
                return (None,None)
        else:
            return (None,None)
        
    

    
    