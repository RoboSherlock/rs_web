package org.knowrob.robosherlock.db;

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
		getIdentifiables();
	}

	void getIdentifiables(){
		if(sceneDocument.containsField("identifiables")){
			this.identifiables = (BasicDBList) sceneDocument.get("identifiables");
			System.out.println("Scene contains: " + this.identifiables.size() + " clusters");
			for (Object cluster:this.identifiables)
			{
				DBObject o = (DBObject)cluster;
				System.out.println(o.containsField("annotations"));
			}
		}
	}
	public int getNumberofClusters()
	{
		return identifiables.size();
	}

}
