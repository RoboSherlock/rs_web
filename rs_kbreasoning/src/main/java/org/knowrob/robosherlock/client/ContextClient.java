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



import designator_integration_msgs.*;

public class ContextClient extends AbstractNodeMain {
	

        ServiceClient<designator_integration_msgs.ResetRSContextRequest,
	designator_integration_msgs.ResetRSContextResponse> setContextServiceClient;
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
			//			String service_name = "/robosherlock/designator_request/single_solution";
			String context_service_name = "/RoboSherlock_openease/designator_request/set_context";
			
			setContextServiceClient = node.newServiceClient(context_service_name, 
					designator_integration_msgs.ResetRSContext._TYPE);
	
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
		designator_integration_msgs.ResetRSContextRequest request = setContextServiceClient.newMessage();	
		request.setNewAe(contextName);
		System.out.println("request message "+ contextName);
		setContextServiceClient.call(request,new ServiceResponseListener<designator_integration_msgs.ResetRSContextResponse>()				
			{
			@Override
			public void onSuccess(designator_integration_msgs.ResetRSContextResponse arg0) {
				System.out.println("success");
				//				System.out.println("Successfully called servidce.");
			}
			@Override
			public void onFailure(RemoteException e) {
				System.out.println("Failed to call the service");
				throw new RosRuntimeException(e);
			}
					    });
	}

	//call RoboSherlock with a timestamp. RS should get the frame at this timestamp and process it
}
