:- module(rs_query_interface,
  [
  do_some_magic/1,
  test_new_interface/1,
  rs_interface/1,
  execute_pipeline/1,
  scene_clusters_count/3,
  detect/2,
  set_context/1,
  query_result/5,
  test/1,
  test_list/1,
  get_list_of_predicates/2
]).

%%%%%%%%%%%%%%%% BEGIN: Java Client calls %%%%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%% END: C++ Interface %%%%%%%%%%%%%%%%%%%%    

%%%%%%%%%%%%%%%% BEGIN: TESTS %%%%%%%%%%%%%%%%%%%% 
do_some_magic([Head|Tail]):-
    length(Tail,Tl),
    print(Head),tab(2),print(Tail),tab(2),print(Tl),nl.
%    test_new_interface(Tail).

test_new_interface(List):-	
    forall(member(M,List),
                is_list(M)->(
                print(M),nl,
                do_some_magic(M)
            )
          ).
    %    is_list(Head)->(
    %        test_new_interface(Head),
    %        length(Tail,Lt),
    %        Lt=1 -> member(Mt,Tail);
    %        test_new_interface(Tail)
    %    );
    %    writef(Head,['-']),print(Tail,[]),
    %    put(10),
    %    length(Tail,LT),print(LT),put(10),
    %    LT>1->test_new_interface(Tail);
    %    member(MT,Tail),
    %writef(MT,['\n']),put(10).


    %member(M,List),
    %is_list(M)->
    %test_new_interface(M,A);
    %print(M).
    %is_list(M),
    %test_new_interface(M);
    %print(M).

test(A):-
    test_new_interface([an, object,[shape,spehere],[size,medium],[logo,'Dr.Oetker'],[color,blue], [location,[a,location,[on,[an, object,[type, table]]]]],[location,[a,location,[next-to,[an,object,[[shape,box],[type,drink]]]]]]]).

test_list(A):-
    flatten([an, object,[shape,spehere],[size,medium],[logo,'Dr.Oetker'],[color,blue], [location,[a,location,[on,[an, object,[type, table]]]]],[location,[a,location,[next-to,[an,object,[[shape,box],[type,drink]]]]]]],A).


get_list_of_predicates([],[]).
get_list_of_predicates([Pred|T],[Pred|Result]):-
	is_predicate(Pred),
	get_list_of_predicates(T,Result).
get_list_of_predicates([ThrowAway|Tail],Result):-
get_list_of_predicates(Tail,Result).


%%%%%%%%%%%%%%%% END: TESTS %%%%%%%%%%%%%%%%%%%% 

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
