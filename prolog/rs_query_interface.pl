:- module(rs_query_interface,
  [
  scene_clusters_count/3,
  detect/2,
  set_context/1,
  query_result/5
]).

client_interface :-
    client_interface(_).

:- assert(rs_interface(fail)).
client_interface(Client) :-
    rs_interface(fail),
    jpl_new('org.knowrob.robosherlock.client.RSClient',[],Client),
    retract(rs_interface(fail)),
    jpl_list_to_array(['org.knowrob.robosherlock.client.RSClient'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities',runRosjavaNode,[Client,Arr],_),
    assert(rs_interface(Client)),!.
       
client_interface(Client) :-
    rs_interface(Client).

context_client_interface :-
    context_client_interface(_).

:- assert(rs_context_interface(fail)).

context_client_interface(Client) :-
    rs_context_interface(fail),
    jpl_new('org.knowrob.robosherlock.client.ContextClient',[],Client),
    retract(rs_context_interface(fail)),
    jpl_list_to_array(['org.knowrob.robosherlock.client.ContextClient'], Arr),  
    jpl_call('org.knowrob.utils.ros.RosUtilities',runRosjavaNode,[Client,Arr],_),
    assert(rs_context_interface(Client)),!.       

context_client_interface(Client) :-
    rs_context_interface(Client).

detect(Query,FrameID):-
    client_interface(Cl),
    jpl_list_to_array(Query,QueryArray),
    jpl_call(Cl,'callService',[QueryArray,FrameID],_),!.
    
set_context(CName):-
    client_interface(Cl),
    jpl_call(Cl,'changeDB',[CName],_),
    context_client_interface(C),	
    jpl_call(C,'callSetContextService',[CName],_).
    
%%count object hypotheses logged in a scene by timestamp and Scene name
scene_clusters_count(Timestamp,Collection,Count):-
    jpl_new('org.knowrob.robosherlock.db.RSMongoWrapper',[Collection],RS),
    jpl_call(RS,'getScene',[Timestamp],SceneObject),
    jpl_call(SceneObject,'getNumberOfClusters',[],Count).		

% Query the results of the database
% Terms are: shape,color,size...for now..add instance
% example: query_result(['size','shape'],['medium','flat'],'1409046631131301703','Scenes_annotated',R).
query_result(Terms,Values,Timestamp,Collection,R):-
    jpl_new('org.knowrob.robosherlock.db.RSMongoWrapper',[Collection],RS),
    jpl_call(RS,'getScene',[Timestamp],SceneObject),
    jpl_list_to_array(Terms,ATerms),
    jpl_list_to_array(Values,AValues),	
    jpl_call(SceneObject,'query',[ATerms,AValues],R).	