# -*- coding: utf-8 -*-
"""
Created on Tue Mar 14 14:01:25 2017

@author: ferenc
"""

import rospkg

from pyswip import Prolog  #,registerForeign

class PySWIPWrapper:
   
    def __init__(self,initPkg):
        self.prolog = Prolog()
        path_to_rosprolog = rospkg.RosPack().get_path('rosprolog')
        self.prolog.consult(path_to_rosprolog+"/prolog/init.pl")
        print(list(self.prolog.query("register_ros_package(knowrob_robosherlock).")))
        print("PySwipWrapper initialized")
           
   
    def scenes(self):
        for i in list(self.prolog.query("knowrob_robosherlock:keyword(A)")):
            print (i)

def main():
    pl = PySWIPWrapper("knowrob_robosherlock")
    pl.scenes()
    
    
if __name__ == "__main__":
    main()   