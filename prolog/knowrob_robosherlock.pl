:- module(knowrob_robosherlock,
    [
        call_robosherlock/2, 
  scene_clusters_count/3,
  compute_annotators/1,
  annotators/1,
  compute_annotator_outputs/2,
  compute_annotator_inputs/2,
  annotator_outputs/2,
  annotator_inputs/2,
  shape_annotators/1,
  type_available/1,
  annotator_in_dependency_chain_of/2,
  dependency_chain_as_set_for_annotator/2,
  dependency_chain_ordering/3,
  ordered_dependency_chain_for_annotator/2,
  annotator_missing_inputs/2,
  annotators_satisfying_atleast_one_input/2,
  annotators_satisfying_direct_inputs_of/2,
  get_required_annotators_for_annotator_list/2,
  annotatorlist_requirements_fulfilled/1,
  get_missing_annotators/2,
  can_inputs_be_provided_for_annotator_list/1,
  build_pipeline/2,
  annotators_for_predicate/2,
  annotators_for_predicates/2,
  build_pipeline_from_predicates/2,
  obj_has_predicate/2,
  predicates_for_object/2
]).

:- rdf_meta
   call_robosherlock(+,+),
   scene_clusters_count(+,+,+).

:- use_module(library('jpl')).

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_srdl).

:- owl_parse('package://knowrob_robosherlock/owl/rs_components.owl').
:- rdf_db:rdf_register_ns(rs_components, 'http://knowrob.org/kb/rs_components.owl#',     [keep(true)]).

call_robosherlock(Response,Timestamp):-
    jpl_new('org.knowrob.robosherlock.client.RSClient',[],Client),
    jpl_list_to_array(['org.knowrob.robosherlock.client.RSClient'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities',runRosjavaNode,[Client,Arr],_),
    jpl_call(Client,'callService',[Timestamp],Response).

scene_clusters_count(Timestamp,Collection,Count):-
    jpl_new('org.knowrob.robosherlock.db.RSMongoWrapper',[Collection],RS),
    jpl_call(RS,'getScene',[Timestamp],SceneObject),
    jpl_call(SceneObject,'getNumberOfClusters',[],Count).		



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pipeline Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
compute_annotators(A) :- owl_subclass_of(A,rs_components:'RoboSherlockComponent'),
                  not(A = 'http://knowrob.org/kb/rs_components.owl#RoboSherlockComponent'), 
                  not(A = 'http://knowrob.org/kb/rs_components.owl#AnnotationComponent'), 
                  not(A = 'http://knowrob.org/kb/rs_components.owl#DetectionComponent'), 
                  not(A = 'http://knowrob.org/kb/rs_components.owl#IoComponent'), 
                  not(A = 'http://knowrob.org/kb/rs_components.owl#PeopleComponent'), 
                  not(A = 'http://knowrob.org/kb/rs_components.owl#SegmentationComponent'). 


% cache the annotators
:- forall(compute_annotators(A), assert(annotators(A)) ).

% Get outputs of Annotator
compute_annotator_outputs(Annotator,Output) :- annotators(Annotator), class_properties(Annotator,rs_components:'perceptualOutput',Output).
% Get inputs of Annotator
compute_annotator_inputs(Annotator,Input) :- annotators(Annotator), class_properties(Annotator,rs_components:'perceptualInputRequired',Input).
% cache outputs/inputs
:- forall(compute_annotator_outputs(A,O), assert(annotator_outputs(A,O)) ).
:- forall(compute_annotator_inputs(A,I), assert(annotator_inputs(A,I)) ).

shape_annotators(A) :- annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationShape' ).
% shape_annotators_old(A) :- owl_has(A,rs_components:factResult,rs_components:'RsAnnotationShape').


% Get every type that can be put out by any annotator
type_available(Output) :- annotator_outputs(_,Output).

% Check if Annotator A is somewhere in the Depedency chain of D.
% This means for example in RoboSherlock, where the CollectionReader should be at
% the first place in every pipeline:
% annotator_in_dependency_chain_of(CollectionReader,SomeShapeAnnotator) should be true.
% annotator_in_dependency_chain_of(SomeShapeAnnotator, CollectionReader) should be false.
% annotator_in_dependency_chain_of(SomeShapeAnnotator, imagePreprocessor) should be false.
% annotator_in_dependency_chain_of(X, Collectionreader) should be false.
%
% Trivial case: A is in the dependency chain of D, if A provides a type that D needs.
annotator_in_dependency_chain_of(A, D) :- 
	annotator_outputs(A,Input),
	annotator_inputs(D, Input).
% Recursive case: A is in the Dependency chain of D, if A provides a Type
% that X needs, and X provides a type that D needs.
annotator_in_dependency_chain_of(A, D) :- 
	annotator_outputs(A,Input),
	annotator_inputs(X, Input),
	annotator_in_dependency_chain_of(X, D).

% calculate the full dependency chain for a given
% Annotator, include the annotator itself. The chain 
% MUST NOT be in the correct order
dependency_chain_as_set_for_annotator(Annotator,S) :-
	L=[Annotator],
	setof(X,annotator_in_dependency_chain_of(X,Annotator),D),
	append(L,D,S); % Either a set of dependencies can be calculated
	S=[]. % or we return an empty list, when no dependencies are present

% AnnotatorA < AnnotatorB when A is somewhere in the beginning
% of the dependency chain of B
dependency_chain_ordering(R, AnnotatorA, AnnotatorB) :-
	annotator_in_dependency_chain_of(AnnotatorA, AnnotatorB) -> R = '<' ; R = '>'.
% Order the output of dependency_chain_as_set_for_annotator in a manner
% that the evaluation order in L is correct.
ordered_dependency_chain_for_annotator(Annotator,L) :-
	dependency_chain_as_set_for_annotator(Annotator,AnnotatorChainSet),
	predsort(dependency_chain_ordering, AnnotatorChainSet, L).


% Can an input never be satisified?
annotator_missing_inputs(Annotator,Missing) :- 
	findall(Input, (annotator_inputs(Annotator, Input),
	not(type_available(Input)) ), Missing).

% Get a list caled AnnotatorSatisfyingInput, that
% includes all annotators that provide _one_ input of Annotator A.
annotators_satisfying_atleast_one_input(Annotator, AnnotatorSatisfyingInput):-
	annotator_inputs(Annotator, Input),
	setof(X, annotator_outputs(X,Input), AnnotatorSatisfyingInput).

% Get a List of Annotators, that provide the required inputs of
% _ALL_ inputs of A
annotators_satisfying_direct_inputs_of(Annotator, AnnotatorSet):-
	setof(X, annotators_satisfying_atleast_one_input(Annotator, X), L),
	flatten(L, AnnotatorSet);
  AnnotatorSet = []. % Return empty set, when a annotator doesn't need any inputs

get_required_annotators_for_annotator_list(AnnotatorList,RequiredAnnotators):-
	maplist(annotators_satisfying_direct_inputs_of, AnnotatorList, List),
	flatten(List, FlattenedList),
	list_to_set(FlattenedList, RequiredAnnotators).

% Check, if all the required annotators of the annotators are in the given list.
% WARNING: This does NOT work if you pass a list, that has unsatisfiable
% input requirements. This means, that the input of an Annotator
% is not the Result of ANY Annotator in the KnowledgeBase.
annotatorlist_requirements_fulfilled(AnnotatorList):-
  get_required_annotators_for_annotator_list(AnnotatorList, ReqA),!,
  % is ReqA a Subset of AnnotatorList? 
  subset(ReqA, AnnotatorList).

% Take a List of Annotators called L, calculate all the required inputs
% and the Annotators that do provide them.
% Add the Annotators to L.
% Repeat, until the size of L doesn't change.
% add_required_annotators_until_inputs_satisfied(AnnotatorList, ResultList).

% Take a List of Annotators, calculate it's dependencies on other
% Annotators and add them to the ResultList.
get_missing_annotators(AnnotatorList, ResultList):-
	maplist(dependency_chain_as_set_for_annotator,AnnotatorList, L),
	flatten(L, FlattenedList),
	list_to_set(FlattenedList, ResultList).

% Check, if the required inputs of the Anntators in AnnotatorList
% can be provided by any of the Annotators in the System.
% If the Annotator doesn't require any inputs, the method will be true.
can_inputs_be_provided_for_annotator_list(AnnotatorList):-
  % check for all members of AnnotatorList
  forall(member(R,AnnotatorList),
    % The Annotator doesn't need any inputs
    (\+ annotator_inputs(R,T) ;
      % or: EVERY input will be provided by some annotator.
      forall(annotator_inputs(R,T), annotator_outputs(_,T))
    )
  ).

% TODO: Consistency Checks! Check the dependency graph for the absence of cycles.
% TODO: Test with multiple inputs 

% ListOfAnnotators: A List of Annotators that should be run. The list does not have to include the necessary dependencies for the Annotators nor must be in the correct order.
% EvaluationList: A List of Annotators that form a complete Pipeline. The Annotators should be in the correct evaluation order
build_pipeline(ListOfAnnotators,EvaluationList):-
  % Are there any requested types that can't be calculated by the system?
  can_inputs_be_provided_for_annotator_list(ListOfAnnotators) ->
    (annotatorlist_requirements_fulfilled(ListOfAnnotators) ->
      % If requirements are already fulfilled, bring everything in the correct
      % order and return
      predsort(dependency_chain_ordering, ListOfAnnotators, EvaluationList);
      % else: get the missing annotators to complete the list and sort it.
      get_missing_annotators(ListOfAnnotators, FullListOfAnnotators),!,
      predsort(dependency_chain_ordering, FullListOfAnnotators, EvaluationList)
    )
  ; write('** WARNING: One or more inputs of the given List of Annotators can not be computed by an Algorithm in the KnowledgeBase **'),fail.


% Map a predefined set of predicates to Annotator Outputs
annotators_for_predicate(shape,A) :-
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationShape' ).
annotators_for_predicate(color,A) :- 
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationSemanticcolor' ).
annotators_for_predicate(size,A) :- 
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGeometry' ).
annotators_for_predicate(location,A) :- 
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationTflocation' ).
annotators_for_predicate(logo,A) :- 
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGoggles' ).
annotators_for_predicate(text,A) :- 
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGoggles' ).
annotators_for_predicate(product,A) :- 
  annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGoggles' ).

annotators_for_predicates(Predicates, A):-
  member(P,Predicates), annotators_for_predicate(P, A).

obj_has_predicate(shape, Obj):- 
  class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),owl_subclass_of(O, knowrob:'SpatialThingTypeByShape').

obj_has_predicate(color, Obj):- 
  class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),owl_subclass_of(O, knowrob:'ColoredThing').

obj_has_predicate(size, Obj):- 
  class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),owl_subclass_of(O, rs_components:'Size').

obj_has_predicate(logo, Obj):- 
  class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),owl_subclass_of(O, rs_components:'Logo').

obj_has_predicate(text, Obj):- 
  class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),owl_subclass_of(O, rs_components:'TextOnObject').

% TODO: Define something for product


predicates_for_object(Obj,Preds):- 
	setof(X,obj_has_predicate(X,Obj),Preds).

% TODO:
% Read in all predicates and look for suitable annotators in the knowledge base.

build_pipeline_from_predicates(ListOfPredicates,Pipeline):-
	setof(X,annotators_for_predicates(ListOfPredicates, X), Annotators),
  build_pipeline(Annotators, Pipeline).
% 
% build_pipeline_from_predicates([shape,color],Pipeline):-
% 	setof(X,shape_annotators(X),ShapeAnnotators),
%   build_pipeline(ShapeAnnotators, Pipeline).

% Reminder on using maplist:
% test(N,R):- R is N*N.
% ?- maplist(test,[3,5,7],X).
% X = [9,25,49].


/*
* Definitions done
*/

:- print('----------\n').
:- print('RSComponents ontology is available under http://knowrob.org/kb/rs_components.owl#\n').
:- print('----------\n').

