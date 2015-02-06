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

		RSMongoWrapper mongoWrapper = new RSMongoWrapper("Scenes_annotated");
		mongoWrapper.getCas("asdd");
	}
}

