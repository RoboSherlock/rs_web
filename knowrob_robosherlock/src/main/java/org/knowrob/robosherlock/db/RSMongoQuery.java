package org.knowrob.robosherlock.db;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.knowrob.robosherlock.db.RSMongoWrapper;


public class RSMongoQuery {
	public RSMongoQuery(String dbname) {
		// TODO Auto-generated constructor stub
	}
	public String SimpleOutput()
	{
		return "Called a function";
	}

	public static void main(String[] args) {

		RSMongoWrapper mongoWrapper = new RSMongoWrapper("Scenes_annotated");
		Scene scene =  mongoWrapper.getScene("1409046631131301703");
		System.out.println("No. of clusters in scene:" + scene.getNumberOfClusters());
//		scene.query("iai_rs.annotation.Geometry","medium");
//		scene.query("iai_rs.annotation.SemanticColor","white");
		
		final HashMap<String,String> query;
		
		query = new HashMap <String,String> ();
		query.put ("iai_rs.annotation.Geometry","medium");
		query.put ("iai_rs.annotation.SemanticColor", "yellow");
		query.put ("iai_rs.annotation.Shape", "round");
		ArrayList<Integer> results = scene.query(query);
		if(results.size()==0)
		{
			System.out.println("no clusters match");
		}
		for(Integer i:results)
		{
			System.out.println("Cluster " + i + " matches");	
		}
		//methods for getting extra information about cluster
		
		
	}
}

