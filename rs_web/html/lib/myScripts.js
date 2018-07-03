// function myFunction() {
//     var button = document.getElementById("myButton");
//     var iframe  = document.getElementById("webgz");
//     if (button.value=="gzweb on") {
// 	button.value = "gzweb off";
//         iframe.style.display="inline";
//     }
//     else {
//     	button.value = "gzweb on";
//         iframe.style.display="none";
//     }	
// }

// function stopRS()
// {
//     var client = new JsonProlog(ros, {});
//     client.jsonQuery("rs_pause(a)",function(result) {
//                         client.finishClient();
//                     });
// }

/*function rs_ros(this){
  
  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });
  
    ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });
    
    var json_prolog = new ROSLIB.Service({
    ros : ros,
    name : '/json_prolog/simple_query',
    serviceType : 'json_prolog_msgs/PrologQuery'
  });
}*/

function myFunction() {
    document.getElementById("myDropdown").classList.toggle("show");
}

function filterFunction() {
    var input, filter, ul, li, a, i;
    input = document.getElementById("myInput");
    filter = input.value.toUpperCase();
    div = document.getElementById("myDropdown");
    a = div.getElementsByTagName("a");
    for (i = 0; i < a.length; i++) {
        if (a[i].innerHTML.toUpperCase().indexOf(filter) > -1) {
            a[i].style.display = "";
        } else {
            a[i].style.display = "none";
        }
    }
}