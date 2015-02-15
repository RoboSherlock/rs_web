package org.knowrob.robosherlock.db;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import com.mongodb.BasicDBList;
import com.mongodb.DBObject;

public class Scene {
	DBObject sceneDocument;
	long timestamp;
	BasicDBList identifiables;
	
	
	class EntryPair{
		public EntryPair(String name2, String value2) {
			this.name=name2;
			this.value= value2;
		}
		String name;
		String value;		
	}
	
	/*entry in DB with the type name of the annotation and field name that contains
	the value of interest*/
	private static final Map<String,String> defTypes;
	static
	{
		defTypes = new HashMap <String,String> ();
		defTypes.put ("iai_rs.annotation.Geometry","size");
		defTypes.put ("iai_rs.annotation.Shape", "shape");
		defTypes.put ("iai_rs.annotation.TFLocation","frame_id");
//		defTypes.put ("iai_rs.annotation.SemanticColor","color");
	}
	
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

	public boolean searchForAnnotation(EntryPair key,  DBObject object, int index)
	{
		if(object.containsField("annotations")){
			BasicDBList annotations = (BasicDBList) object.get("annotations");
			for (Object a:annotations){
				DBObject annotation = (DBObject)a;
				if(annotation.containsField("_type")){
					String type = (String)annotation.get("_type");
					if(type.equals(key.name)){
					
						
						if(!key.name.equals("iai_rs.annotation.SemanticColor")){
							String value = (String)	annotation.get(defTypes.get(type));
							if(value.equals(key.value)){
								System.out.println("Cluster " + index + " has " + key.name +" " +key.value);
								return true;
							}
							else 
								return false;
						}
						else
						{
							BasicDBList colors = (BasicDBList) annotation.get("color");
							BasicDBList ratios = (BasicDBList) annotation.get("ratio");
							if (colors.size() != ratios.size())
							{
								return false;
							}
							else{
								for (int i =0; i<colors.size(); ++i){
									String color = (String)colors.get(i);
									Double ratio = (Double)ratios.get(i);
									if(ratio>0.20 && color.equals(key.value)){
										System.out.println("Cluster " + index + " has " + key.name +" "+ key.value);
										return true;
									}
								}
							return false;	
							}
						}
					}
				}
			}
		}
		return false;
	}
	
	public ArrayList<Integer> query(ArrayList<EntryPair> Keys){
		int cIndex = 0;
		ArrayList<Integer> clustersFound = new ArrayList<Integer>();
		for (Object cluster:this.identifiables)
		{
			boolean found=true;
			for (EntryPair key:Keys)
			{
				DBObject o = (DBObject)cluster;
				if(!searchForAnnotation(key, o, cIndex)){
					found=false;
				}
				
			}
			if(found){
				clustersFound.add((Integer) cIndex);
			}
			cIndex++;	
		}
		return clustersFound;
	}
	public boolean query(String name, String value){
		ArrayList<EntryPair> queryItem = new ArrayList<EntryPair>();
		queryItem.add(new EntryPair(name,value));
		ArrayList<Integer> results = query(queryItem);
		boolean found=false;
		if(results.size()==0)
		{
			System.out.println("Found no results");
		}
		else
		{
			for (Integer i:results)
				System.out.println("Cluster " + i);
			found = true;
		}
		return found;
	}
	public ArrayList<Integer> query(HashMap<String,String> query)
	{
		ArrayList<Integer> resultClusterIndices = new ArrayList<Integer>();
		ArrayList<EntryPair> queryItems = new ArrayList<EntryPair>();
		Set<String> keys = 	query.keySet();
		for (String key:keys){
			queryItems.add(new EntryPair(key, query.get(key)));
		}
		resultClusterIndices = query(queryItems);
		return resultClusterIndices;
	}
}