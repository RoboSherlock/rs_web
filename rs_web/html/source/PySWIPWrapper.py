# -*- coding: utf-8 -*-
"""
Created on Tue Mar 14 14:01:25 2017

@author: ferenc
"""

import rospkg

from pyswip import Prolog  #,registerForeign

class PySWIPWrapper(object):

    prolog = Prolog()

    def __init__(self,initPkg):

        path_to_rosprolog = rospkg.RosPack().get_path('rosprolog')
        PySWIPWrapper.prolog.consult(path_to_rosprolog+"/prolog/init.pl")
        try:
            print(list(PySWIPWrapper.prolog.query("register_ros_package("+initPkg+").")))
        except:
            print ('%s ROS package does not exist.')
        print("PySwipWrapper initialized")
           
   
    def __end__(self):
        print('End of the Road')


    def scenes(self):
        for i in list(PySWIPWrapper.prolog.query("knowrob_robosherlock:keyword(A)")):
            print (i)

def main():
    pl = PySWIPWrapper("knowrob_robosherlock")
    try:
        pl.scenes()
    except:
        print('Computer says NOoooo')
    print 'Done'
    pl.__end__()
if __name__ == "__main__":
    main()
