package org.knowrob.robosherlock.db;

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
		System.out.println("start");	

		RSMongoWrapper mongoWrapper = new RSMongoWrapper("Scenes");
		Set<String> collections = mongoWrapper.db.getCollectionNames();
		for( String c:collections)
		{
			System.out.println(c);
		}
		
	}
}

