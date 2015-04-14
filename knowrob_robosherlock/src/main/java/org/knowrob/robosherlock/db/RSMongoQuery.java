package org.knowrob.robosherlock.db;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.knowrob.robosherlock.db.RSMongoWrapper;
//import org.opencv.core.Core;


public class RSMongoQuery {
	public RSMongoQuery(String dbname) {
		// TODO Auto-generated constructor stub
	}
	public String SimpleOutput()
	{
		return "Called a function";
	}
	//testbed
	public static void main(String[] args) {
//		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		RSMongoWrapper mongoWrapper = new RSMongoWrapper("RSOpenEaseData");
		Scene scene =  mongoWrapper.getScene("1426179784789117024");
		System.out.println(mongoWrapper.timestamps.get(1));
//		System.out.println("No. of clusters in scene:" + scene.getNumberOfClusters());
		
//		final HashMap<String,String> query;
		
//		query = new HashMap <String,String> ();
//		query.put ("iai_rs.annotation.Geometry","medium");
//		query.put ("iai_rs.annotation.SemanticColor", "yellow");
////		query.put ("iai_rs.annotation.Shape", "box");
//		
//		ArrayList<Integer> results = scene.query(query);
//		if(results.size()==0)
//		{
//			System.out.println("no clusters match");
//		}
//		for(Integer i:results)
//		{
//			System.out.println("Cluster " + i + " matches");	
//		}
//
//		mongoWrapper.getRGB("1409046631131301703");
//		mongoWrapper.getDepth("1409046631131301703");
	}
}

