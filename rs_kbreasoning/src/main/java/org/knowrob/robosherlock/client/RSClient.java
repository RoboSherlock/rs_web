package org.knowrob.robosherlock.client;

import java.util.ArrayList;
import java.util.List;

import org.knowrob.robosherlock.db.RSMongoWrapper;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import com.mongodb.MongoWaitQueueFullException;

import designator_integration_msgs.*;

public class RSClient extends AbstractNodeMain {
	
	ServiceClient<designator_integration_msgs.DesignatorCommunicationRequest,
	designator_integration_msgs.DesignatorCommunicationResponse> serviceClient;
	
        int globalTSIndex;
	public ConnectedNode node;
	RSMongoWrapper mongoWrapper;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_robosherlock/rs_client");
	}
	//  rosjava_test_msgs.AddTwoInts._TYPE
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.node = connectedNode;
		mongoWrapper = new RSMongoWrapper("kitchen");
		globalTSIndex = 0;
		// wait for node to be ready
		try {
			while(node == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}


		try {
			//service name change required in RoboSherlock (to not collide with other RS instances running on the same machine
			String service_name = "/RoboSherlock/designator_request/single_solution";
			//			String service_name = "/robosherlock/designator_request/single_solution";
			serviceClient = node.newServiceClient(service_name, 
					designator_integration_msgs.DesignatorCommunication._TYPE);
	
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}

	}

	//call RoboSherlock with a timestamp. RS should get the frame at this timestamp and process it
	public void callServiceWithTimestamp(String[] queryItems, String timestamp)
	{
		// wait for node to be ready
		try {
			while(serviceClient == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		DesignatorCommunicationRequest request = serviceClient.newMessage();		
		DesignatorRequest req =  node.getTopicMessageFactory().newFromType(DesignatorRequest._TYPE);
		Designator desig =  node.getTopicMessageFactory().newFromType(Designator._TYPE);
		desig.setType(0);

		ArrayList<KeyValuePair> kvp_list = new ArrayList<KeyValuePair>();
		addKeyValuePair("TIMESTAMP", timestamp, kvp_list);
		parseQuery(queryItems, kvp_list);
		desig.setDescription(kvp_list);
		req.setDesignator(desig);
		request.setRequest(req);
		System.out.println("Calling service with timestamp t="+timestamp);
		serviceClient.call(request, new ServiceResponseListener<designator_integration_msgs.DesignatorCommunicationResponse>() 
				{
			@Override
			public void onSuccess(DesignatorCommunicationResponse arg0) {
				//				System.out.println("Successfully called servidce.");
			}
			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
				});
		//		return "Success";
	}

	public void callService(String[] queryItems, String frameidx)
	{
		// wait for node to be ready
		try {
			while(serviceClient == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		DesignatorCommunicationRequest request = serviceClient.newMessage();		
		DesignatorRequest req =  node.getTopicMessageFactory().newFromType(DesignatorRequest._TYPE);
		Designator desig =  node.getTopicMessageFactory().newFromType(Designator._TYPE);
		desig.setType(0);

		String[] kvp = frameidx.split(":");
		if(kvp.length !=2)
		{
			System.out.println("Frame id not formated correctly. Usecase: 'frame:x'; incrementing ts index by one");
			globalTSIndex = (globalTSIndex+1) % mongoWrapper.timestamps.size();
			System.out.println("globalTSIndex: " + globalTSIndex);
		}
		else
		{
			globalTSIndex = Integer.parseInt(kvp[1])%mongoWrapper.timestamps.size();
		}
		ArrayList<KeyValuePair> kvp_list = new ArrayList<KeyValuePair>();

		String timestamp = mongoWrapper.timestamps.get(globalTSIndex);
		addKeyValuePair("TIMESTAMP", timestamp, kvp_list);
		parseQuery(queryItems, kvp_list);
		desig.setDescription(kvp_list);
		req.setDesignator(desig);
		request.setRequest(req);
		System.out.println("Calling service with timestamp t="+timestamp);
		serviceClient.call(request, new ServiceResponseListener<designator_integration_msgs.DesignatorCommunicationResponse>() 
				{
			@Override
			public void onSuccess(DesignatorCommunicationResponse arg0) {
				//				System.out.println("Successfully called servidce.");
			}
			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
				});
	}
	//		return "Success";


	boolean parseQuery(String[] queryItems, ArrayList<KeyValuePair> list)
	{
		for (int i=0; i<queryItems.length; ++i)
		{

			if(queryItems[i].contains(":"))
			{
				String[] kvp = queryItems[i].split(":");
				if(kvp.length != 2 )
				{
					System.out.println("Malformed key value pair. for the query like this: ['shape:box','color:blue','location:table-top']");
				}
				else 
				{
					addKeyValuePair(kvp[0].toUpperCase(),kvp[1],list);
				}
			}
			else
			{
				addKeyValuePair(queryItems[i].toUpperCase(),"",list);
			}
		}
		return true;	
	}

	void addKeyValuePair(String Key, String Value, ArrayList<KeyValuePair> list)
	{
		KeyValuePair kvp = node.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
		kvp.setKey(Key);
		kvp.setValueString(Value);
		//this will not allow hierarchical tree structures, but will have to do for now
		kvp.setId(list.size()+1);
		list.add(kvp);
	}
	public boolean changeDB(String dbName){
		globalTSIndex = 0;
		mongoWrapper.changeDB(dbName);
		return true;
	}
}
