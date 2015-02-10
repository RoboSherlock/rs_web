package org.knowrob.robosherlock.db;

import java.util.HashMap;

import com.mongodb.BasicDBList;
import com.mongodb.DBObject;

public class Scene {
	DBObject sceneDocument;
	long timestamp;
	BasicDBList identifiables;

	Scene(DBObject sceneD, String timestamp)
	{
		this.sceneDocument = sceneD;
		try{
			this.timestamp = Long.parseLong(timestamp);
		}
		catch(NumberFormatException e){
			System.out.println("Timestamp not a number");
		}
		getIdentifiablesAndAnnotations();
	}

	public void getIdentifiablesAndAnnotations(){
		if(sceneDocument.containsField("identifiables")){
			this.identifiables = (BasicDBList) sceneDocument.get("identifiables");
			System.out.println("Scene contains: " + this.identifiables.size() + " clusters");
		}
	}
	public int getNumberOfClusters(){
		return identifiables.size();
	}

	public void searchForAnnotation(String key)
	{
		int cIndex = 0;
		for (Object cluster:this.identifiables)
		{
			DBObject o = (DBObject)cluster;
			if(o.containsField("annotations")){
				BasicDBList annotations = (BasicDBList) o.get("annotations");
				for (Object a:annotations){
					DBObject annotation = (DBObject)a;
					if(annotation.containsField("_type")){
						String type = (String)annotation.get("_type");
						if(type.equals(key)){
							System.out.println("Cluster " + cIndex + " has " + key);	
						}
						
					}
				}
			}
			cIndex++;
		}
	}

}
