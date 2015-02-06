package org.knowrob.robosherlock.client;

import java.util.List;

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

public class RSClient extends AbstractNodeMain {

	ServiceClient<designator_integration_msgs.DesignatorCommunicationRequest,
	designator_integration_msgs.DesignatorCommunicationResponse> serviceClient;
	
	public ConnectedNode node;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_robosherlock/rs_client");
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
			serviceClient = node.newServiceClient("RoboSherlock_ferenc/trigger_uima_pipeline", 
					designator_integration_msgs.DesignatorCommunication._TYPE);
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}

	}
	//call RoboSherlock with a timestamp. RS should get the frame at this timestamp and process it
	public String callService(String timestamp)
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
		KeyValuePair k1 = node.getTopicMessageFactory().newFromType(KeyValuePair._TYPE);
		desig.setType(2);
		k1.setKey("TIMESTAMP");
		k1.setValueString(timestamp);
		req.setDesignator(desig);
		request.setRequest(req);
		System.out.println("Calling service with timestamp t="+timestamp);
		serviceClient.call(request, new ServiceResponseListener<designator_integration_msgs.DesignatorCommunicationResponse>() 
				{
			@Override
			public void onSuccess(DesignatorCommunicationResponse arg0) {
				System.out.println("Successfully called servidce.");
			}
			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
				});
		return "Success";
	}
}