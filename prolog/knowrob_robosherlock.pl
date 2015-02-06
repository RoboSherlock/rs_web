:- module(knowrob_robosherlock,
    [
        start_robosherlock/1    
]).

:- use_module(library('jpl')).

:- rdf_meta
   start_robosherlock(+).

start_robosherlock(Task):-
    jpl_new('org.knowrob.robosherlock.db.Client',[],Client),
    jpl_list_to_array(['org.knowrob.robosherlock.db.Client'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities',runRosjavaNode,[Client,Arr],_),
    jpl_call(Client,'callService',[],Task).
