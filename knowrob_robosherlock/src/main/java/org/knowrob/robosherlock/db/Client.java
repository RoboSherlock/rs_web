package org.knowrob.robosherlock.db;

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

public class Client extends AbstractNodeMain {

  ServiceClient<designator_integration_msgs.DesignatorCommunicationRequest,
		designator_integration_msgs.DesignatorCommunicationResponse> serviceClient;
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("knowrob_robosherlock/rs_client");
  }
//  rosjava_test_msgs.AddTwoInts._TYPE
  @Override
  public void onStart(final ConnectedNode connectedNode) {
   
    try {
      serviceClient = connectedNode.newServiceClient("RoboSherlock_ferenc/trigger_uima_pipeline", 
    		  designator_integration_msgs.DesignatorCommunication._TYPE);
    } catch (ServiceNotFoundException e) {
      throw new RosRuntimeException(e);
    }

  }
  public String callService()
  {
    final String returnValue="something";
	designator_integration_msgs.DesignatorCommunicationRequest request = serviceClient.newMessage();
	DesignatorRequest des_req = null;
//	Designator d = null;
//	List<KeyValuePair> kvp_list;
//	KeyValuePair kvKeyValuePair = null;
//	kvKeyValuePair.setKey("asd");
//	kvKeyValuePair.setId(0);
//	kvKeyValuePair.setValueFloat(0.01);
//	kvp_list.(kvKeyValuePair);
//	d.setDescription(kvp_list);
//	des_req.setDesignator(d);
//	request.setRequest(des_req);
//    request.setRequest(null);
    serviceClient.call(request, new ServiceResponseListener<designator_integration_msgs.DesignatorCommunicationResponse>() 
    {

	 @Override
	 public void onSuccess(DesignatorCommunicationResponse arg0) {
		System.out.println("Successfully called servidce");
	 }
	 @Override
     public void onFailure(RemoteException e) {
       throw new RosRuntimeException(e);
     }
    });
	  return returnValue;
  }
}