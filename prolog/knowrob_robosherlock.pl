:- module(knowrob_robosherlock,
    [
        call_robosherlock/2    
]).

:- use_module(library('jpl')).

:- rdf_meta
   call_robosherlock(+,+).

call_robosherlock(Response,Timestamp):-
    jpl_new('org.knowrob.robosherlock.client.RSClient',[],Client),
    jpl_list_to_array(['org.knowrob.robosherlock.client.RSClient'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities',runRosjavaNode,[Client,Arr],_),
    jpl_call(Client,'callService',[Timestamp],Response).
