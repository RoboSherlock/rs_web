# rs_openease

## Perception Planning
RoboSherlock uses Apache UIMA () to process the incoming sensor data with image processing algorithms, called experts. 
The selection and ordering of these experts (e.g. a 'pipeline') is usally hard coded by the developer of a robotic system. 
With RoboSherlock, you can automatically generate these pipelines based on the query you send to RoboSherlock and an Ontology about all implemented experts.
For example, you can ask the system to create a pipeline to retrieve the color and shape attribute of objects in front of your camera:
```
?- build_pipeline_from_predicates([shape,color],S).
S = [rs_components:'CollectionReader',rs_components:'NormalEstimator',
rs_components:'ImagePreprocessor',rs_components:'PointCloudClusterExtractor',
rs_components:'FlatObjectAnnotator',rs_components:'SacModelAnnotator',
rs_components:'PrimitiveShapeAnnotator',rs_components:'ClusterColorHistogramCalculator'].
```
### Unit tests
We've implemented unit tests for the pipeline generation which can be found in prolog/knowrob_robosherlock.plt. These tests can be executed with:
    rosrun rosprolog rosprolog-test knowrob_robosherlock
The test may also be a good entry point for you to see how you can use the pipeline planning for your own projects.
