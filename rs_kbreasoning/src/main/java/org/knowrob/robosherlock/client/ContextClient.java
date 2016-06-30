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



import iai_robosherlock_msgs.*;

public class ContextClient extends AbstractNodeMain {
	

        ServiceClient<iai_robosherlock_msgs.SetRSContextRequest,
		      iai_robosherlock_msgs.SetRSContextResponse> setContextServiceClient;
        int globalTSIndex;
	public ConnectedNode node;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_robosherlock/rs_context_client");
	}
	//  rosjava_test_msgs.AddTwoInts._TYPE
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.node = connectedNode;
		// wait for node to be ready
		try {
			while(node == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		try {	
			String context_service_name = "/RoboSherlock_openease/set_context";
			
			setContextServiceClient = node.newServiceClient(context_service_name, 
					iai_robosherlock_msgs.SetRSContext._TYPE);
	
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}

	}
    public void callSetContextService(String contextName)
	{
		try {
			while(setContextServiceClient == null) {
				System.out.println("Waiting for service client");
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			
			e.printStackTrace();
		}
		iai_robosherlock_msgs.SetRSContextRequest request = setContextServiceClient.newMessage();	
		request.setNewAe(contextName);
		System.out.println("request message "+ contextName);
		setContextServiceClient.call(request,new ServiceResponseListener<iai_robosherlock_msgs.SetRSContextResponse>()				
			{
			@Override
			public void onSuccess(iai_robosherlock_msgs.SetRSContextResponse arg0) {
				System.out.println("success");
			}
			@Override
			public void onFailure(RemoteException e) {
				System.out.println("Failed to call the service");
			}
					    });
	}

	//call RoboSherlock with a timestamp. RS should get the frame at this timestamp and process it
}
