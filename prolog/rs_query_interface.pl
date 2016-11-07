:- module(rs_query_interface,
  [
  rs_interface/1,
  execute_pipeline/1,
  scene_clusters_count/3,
  detect/1,
  detect/2,
  set_context/1,
  query_result/5,
  get_list_of_predicates/2, 
  parse_description/2
]).

%%%%%%%%%%%%%%%% BEGIN: Java Client calls %%%%%%%%%%%%%%%%%%%%

client_interface :-
    client_interface(_).

:- assert(rs_java_interface(fail)).

client_interface(Client) :-
    rs_java_interface(fail),
    jpl_new('org.knowrob.robosherlock.client.RSClient',[],Client),
    retract(rs_java_interface(fail)),
    jpl_list_to_array(['org.knowrob.robosherlock.client.RSClient'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities',runRosjavaNode,[Client,Arr],_),
    assert(rs_java_interface(Client)),!.
       
client_interface(Client) :-
    rs_java_interface(Client).

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

%%%%%%%%%%%%%%%% END: Java Client calls %%%%%%%%%%%%%%%%%%%%    


%%%%%%%%%%%%%%%% BEGIN: C++ Interface %%%%%%%%%%%%%%%%%%%%  
rs_interface :-
   rs_interface(_).

:- assert(rs_interf(fail)).

rs_interface(Client) :-
   rs_interf(fail),
   init_rs(kitchen,Client),
   retract(rs_interf(fail)),
   assert(rs_interf(Client)),!.
    
rs_interface(Cl):-
   rs_interf(Cl).

execute_pipeline(P):-
   rs_interface(Cl),
   process_once(Cl).

designator(location).
designator(object).

keyword(shape).
keyword(size).
keyword(type).
keyword(color).
keyword(on).
keyword(next-to).


add_kvp(Key,Value,D):-
    keyword(Key),
    cpp_add_kvp(Key,Value,D).

add_kvp([],D).
add_kvp([Head|Tail],D):-
    length(Head, Hl),
    (Hl=2->nth1(1,Head,Key),nth1(2,Head,Value),
	   add_kvp(Key,Value,D);
    designator_type(Head),parse_nested_description(Head,D)
    ),
    add_kvp(Tail,D).

parse_nested_description([A,B|Tail],D):-
    designator_type([A,B],T),
    cpp_init_kvp(D,T,KVP),
    add_kvp(Tail,KVP).

parse_description([ A,B | Tail],D):-
    designator_type([A,B],T),
    cpp_add_designator(T,D), %shoudl return a designator Object..prefer C
    add_kvp(Tail,D).%add Kvps-to this designator

designator_type([ A,B | T ] ):-
    designator_type([A,B],N).

designator_type([an,object],'object').
designator_type([a,location],'location').

detect(List):-
    parse_description(List,D),
    cpp_print_desig(D).


%%%%%%%%%%%%%%%% END: C++ Interface %%%%%%%%%%%%%%%%%%%%    

get_list_of_predicates([],[]).
get_list_of_predicates([Pred|T],[Pred|Result]):-
	is_predicate(Pred),
	get_list_of_predicates(T,Result).
get_list_of_predicates([ThrowAway|Tail],Result):-
get_list_of_predicates(Tail,Result).



%%%%%%%%%%%%%%%% BEGIN: Java Result Queries%%%%%%%%%%%%%%%%%%%%
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
    
%%%%%%%%%%%%%%%% END: Java Result Queries%%%%%%%%%%%%%%%%%%%%    
