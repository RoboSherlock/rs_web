:- module(rs_query_interface,
  [
  rs_query_keyword/1,
  rs_interface/2,
  rs_pause/1,
  rs_stop/1,
  execute_pipeline/1,
  detect_json/1,
  detect/1,
  get_list_of_predicates/2, 
  parse_description/2,
  detect_new/2
]).

%%%%%%%%%%%%%%%% BEGIN: C++ Interface %%%%%%%%%%%%%%%%%%%%
%%Queries written using this interface need a sanity check
%%e,g,. spatial relations do not make sense inside a color determiner 
rs_interface :-
   rs_interface(_).

:- assert(rs_interf(fail)).

rs_interface(Client,Ae) :-
   rs_interf(fail),
   cpp_init_ae(Client,Ae),
   retract(rs_interf(fail)),
   assert(rs_interf(Client)),!.
    
rs_interface(Cl):-
   rs_interf(Cl).


execute_pipeline(_):-
   rs_interface(Cl),
   cpp_process_once(Cl).

rs_pause(A):-
   cpp_rs_pause(A).

rs_stop:-
   rs_interf(Cl),
   cpp_stop_rs(Cl),
   assert(rs_interf(fail)).

rs_clear_ae:-
   rs_interf(Ae),
   cpp_remove_ae(Ae),
   assert(rs_interf(fail)).

%defs for syntax checks
designator_type([an,object],'object').
designator_type([an,obj],'object').
sesignator_type([the,object],'object').
designator_type([a,location],'location').

%defs for designator types
designator(location).
designator(object).

%keywords available
rs_query_keyword(shape).
rs_query_keyword(detection).
rs_query_keyword(class).
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

% pars the designator given by detect
parse_description([ A,B | Tail],D):-
    designator_type([A,B],T),
    cpp_add_designator(T,D), 
    add_kvp(Tail,D).

designator_type([ A,B | T ] ):-
    designator_type([A,B],N).


detect(List):-
    rs_interface(A),
    parse_description(List,D),
    thread_create((cpp_print_desig(D),
    cpp_process_once(D),
    cpp_delete_desig(D)),Th,[]).
     %thread_join(Th,Status).

detect_json(Json):-
    rs_interface(A),
    cpp_make_designator(Json,Desig),
    cpp_process_once(Desig).

%%%%%%%%%%%%%%%%%%%%% NEW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

get_keys([],[]).
get_keys([H|T],L1):-
        rs_query_keyword(H),L1=[H|T1],get_keys(T,T1);
        get_keys(T,L1).


rs_pipeline_from_query(Q,P):-
    get_keys(Q,Keys),
    build_single_pipeline_from_predicates(Keys,P),!.

detect_new(List,Pipeline):-
    flatten(List,Lf),
    rs_pipeline_from_query(Lf,Pipeline).

%%%%%%%%%%%%%%%%%%%%% %%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%% END: C++ Interface %%%%%%%%%%%%%%%%%%%%    

get_list_of_predicates([],[]).
get_list_of_predicates([Pred|T],[Pred|Result]):-
	is_predicate(Pred),
	get_list_of_predicates(T,Result).
get_list_of_predicates([ThrowAway|Tail],Result):-
get_list_of_predicates(Tail,Result).



    
%%%%%%%%%%%%%%%% END: Java Result Queries%%%%%%%%%%%%%%%%%%%%    
