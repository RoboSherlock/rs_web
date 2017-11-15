:- module(rs_query_interface,
  [
  rs_query_keyword/1,
  rs_interface/2,
  rs_pause/1,
  rs_stop/1,
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
%%Queries written using this interface need a sanity check
%%e,g,. spatial relations don't make sense inside a color determiner 
rs_interface :-
   rs_interface(_).

:- assert(rs_interf(fail)).

rs_interface(Client,Ae) :-
   rs_interf(fail),
   cpp_init_rs(Ae,Client),
   retract(rs_interf(fail)),
   assert(rs_interf(Client)),!.
    
rs_interface(Cl):-
   rs_interf(Cl).


execute_pipeline(_):-
   rs_interface(Cl),
   process_once(Cl).

rs_pause(A):-
   cpp_rs_pause(A).

rs_stop(A):-
   rs_interf(Cl),
   cpp_stop_rs(A),
   assert(rs_interf(fail)).

%defs for syntax checks
designator_type([an,object],'object').
designator_type([an,obj],'object').
designator_type([the,object],'object').
designator_type([a,location],'location').

%defs for designator types
designator(location).
designator(object).

%keywords available
rs_query_keyword(shape).
rs_query_keyword(detection).
rs_query_keyword(size).
rs_query_keyword(type).
rs_query_keyword(color).
rs_query_keyword(cad-model).
rs_query_keyword(volume).
rs_query_keyword(contains).
rs_query_keyword(timestamp).
rs_query_keyword(handle).

% for simplifying query writing spatial relation can also be keyword

spatial_relation(on).
spatial_relation(in).
spatial_relation(next-to).
spatial_relation(left-of).
spatial_relation(right-of).
spatial_relation(behind).
spatial_relation(in-front-of).

%keyword(A):-
%  spatial_relation(A).

% check if key can exist and add it to designator
add_kvp(Key,Value,D):-
    rs_query_keyword(Key),
    cpp_add_kvp(Key,Value,D).

% handle case when key hints at a nested designator
add_kvp(Key,Value,D):-
    designator(Key),
    parse_nested_description(Value,D).

% needed to manage 'on' as a spatial relation
add_kvp(Key,Value,D):-
    spatial_relation(Key),
    is_list(Value),
    cpp_init_kvp(D,Key,Kvp),
    parse_nested_description(Value,Kvp).

% if query term following spatial relation is not a description (list in our case)
% add the key value pair
add_kvp(Key,Value,D):-
    spatial_relation(Key),
    \+is_list(Value),
    cpp_add_kvp(Key,Value,D).
%return true once List is empty

add_kvp([],D).

%main rule for adding kvps
add_kvp([Head|Tail],D):-
    length(Head, Hl),
    (Hl=2->nth1(1,Head,Key),
	   nth1(2,Head,Value),
	   add_kvp(Key,Value,D);
    	   designator_type(Head),
	   parse_nested_description(Head,D)
    ),
    add_kvp(Tail,D).

% add a nested desig to the main obj-designator
parse_nested_description([A,B|Tail],D):-
    designator_type([A,B],T),
    cpp_init_kvp(D,T,KVP),
    add_kvp(Tail,KVP).

% parse the designator given by detect
parse_description([ A,B | Tail],D):-
    designator_type([A,B],T),
    cpp_add_designator(T,D), 
    add_kvp(Tail,D).

designator_type([ A,B | T ] ):-
    designator_type([A,B],N).


detect(List):-
    rs_interface(A),
    parse_description(List,D),
    cpp_print_desig(D),
    cpp_process_once(D),
    cpp_delete_desig(D).


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
