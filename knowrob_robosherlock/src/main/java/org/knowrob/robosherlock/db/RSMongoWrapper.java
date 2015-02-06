package org.knowrob.robosherlock.db;

import java.net.UnknownHostException;
import java.util.List;

import com.mongodb.BasicDBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.MongoClient;
import com.mongodb.QueryBuilder;

public class RSMongoWrapper {
	
	/*maybe we need a containerClass for the Scene..we will see*/
	public class RSDBEntry{
		DBObject scene_;
		DBObject rgb_;
		DBObject depth_;
		DBObject timestamp_;
		DBObject camInfo_;
	}
	
	private MongoClient mongoClient;
	public DB db;
	RSMongoWrapper(String db_name)
	{
		String host = "localhost";
		int port = 27017;
		try {
			mongoClient = new MongoClient(host, port);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			System.out.println("EXCEPTION!!!");
			e.printStackTrace();
		}
		System.out.println("Connected to mongoDB");
		db = mongoClient.getDB(db_name);
		DBCollection cascol = db.getCollection("cas");
		//get all collections
		collections = new HashMap<String, DBCollection>();
		collections.put("cas", db.getCollection("cas"));
		collections.put("scene", db.getCollection("scene"));
		collections.put("depth_image_as_int", db.getCollection("depth_image_as_int"));
		collections.put("rgb_image_hires", db.getCollection("rgb_image_hires"));
		collections.put("camera_info_hires", db.getCollection("camera_info_hires"));
	}

	public void getCas(String timestamp)
	{
		DBCollection cas = (DBCollection)collections.get("cas");
		System.out.println("Docuemnts in CAS collection:"  + cas.count());
		//		fields.put("PhoneNumber", "NumberLong(" + textField_4.getText() + ")");/"NumberLong(1409046631131301703)"
		BasicDBObject query = new BasicDBObject("_timestamp",Long.parseLong("1409046631131301703"));
		//		query.append("_timestamp", val)
		DBCursor cursor = cas.find(query);
		System.out.println("Found : "+ cursor.count());
		if (cursor.size()!=1)
			System.out.println("Database is corrupt!");
		else if(cursor.hasNext())
		{
			DBObject casEntry = cursor.next();
			System.out.println(casEntry.containsField("scene"));
			ObjectId sceneID = (ObjectId)casEntry.get("scene");
			
			BasicDBObject q = new BasicDBObject();
		    query.put("_id", sceneID);
		    DBCollection scene = (DBCollection)collections.get("scene");
		    DBObject dbObj = scene.findOne(q);
		    System.out.println(dbObj);
		}
		cursor.close();
	}

}


