:- module(rs_similar_objects,
  [
  rs_object_candidates/1,
  rdf_all_similar/3
]).


rdf_all_similar(Class, Super, MostSim) :-
  findall([A, D], (rdfs_subclass_of(A, Super),
                   rdf_wup_similarity(A, Class, D)), Dists),
  predsort(compare_inference_probs, Dists, MostSim).


rs_object_candidates([]).

rs_object_candidates([Head|Tail]):-
  is_valid_candidate(Head,Obj),
  rs_object_candidates(Tail).

is_valid_candidate(List,Obj):-
  length(List,Ll),
  (Ll=2 -> nth1(1,List,Obj),
           owl_class_properties(Obj,knowrob:'pathToCadModel',_)
  ).
