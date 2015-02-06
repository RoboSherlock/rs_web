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
	}
	
}


