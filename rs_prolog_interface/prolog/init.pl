%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_robosherlock).

:- use_module(library(rs_query_interface)).
:- load_foreign_library('rs_prologrulescpp.so').

:- owl_parser:owl_parse('package://rs_prolog_interface/owl/rs_objects.owl').
:- rdf_db:rdf_register_prefix(rs_objects, 'http://knowrob.org/kb/rs_objects.owl#', [keep(true)]).


