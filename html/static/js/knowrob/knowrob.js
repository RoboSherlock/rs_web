function Knowrob(options){
    var that = this;
  
    // The index to the currently active history item
    // history items are saved on the server and queried using AJAX
    var historyIndex = -1;
    
    // Names of prolog predicates and modules for auto completion
    var prologNames;

    // global ROS handle
    var ros = undefined;
    
    // annotator changer service
    var annotatorResultsControl = undefined;
    
    // URL for ROS server
    var rosURL = options.ros_url || 'ws://localhost:9090';
    
    var isConnected = false;
    var isPrologConnected = false;
    var isRegistered = false;

    // global jsonprolog handle
    var prolog;
    
    // the 3d canvas
    var rosViewer;
    
    // keep aliva message publisher
    var keepAlive;
    
    // URL to JSON file that contains episode data
//    var episodeURL = options.episode_url;
//     var episodeURL = 'queriesForRoboSherlock.json'
  
    var episodeURL = [ 'static/queries/queriesForEASE.json','queriesForRoboSherlock.json'];
    // Parsed episode data file
    var episodeData = undefined;
    
    var mngDBName = options.mngDBName;
    
    // The topic where the canvas publishes snapshots
    var snapshotTopic;
    
    var meshPath  = options.meshPath || '/static/meshes/iai_maps';

    // URL for rosauth token retrieval
    var authURL  = options.auth_url || '/wsauth/v1.0/by_session';
    
    // configuration of div names
    var canvasDiv     = options.canvas_div || 'markers'
    var designatorDiv = options.designator_div || 'designator'
    var pictureDiv    = options.picture_div || 'mjpeg'
    var historyDiv    = options.history_div || 'history'
    var libraryDiv    = options.library_div || 'examplequery'
    var libSelectDiv  = options.lib_sel_div || 'querylistselector'
    var queryDiv      = options.query_div || 'user_query'
    var nextButtonDiv = options.next_button_div || 'btn_query_next'
    var annotatorListDiv = options.annotator_list_div || 'myDropdown'
    
    var imageWidth = function() { return 0.0; };
    var imageHeight = function() { return 0.0; };
    
    var background = options.background || '#FFFFFF';
    var near = options.near || 0.01;
    var far = options.far || 1000.0;
    
    this.connect = function () {
      ros = new ROSLIB.Ros({url : rosURL});
      ros.on('connection', function() {
          that.isConnected = true;
          console.log('Connected to websocket server.');
          that.registerNodes();
      });
      ros.on('close', function() {
          that.ros = undefined;
          setTimeout(that.connect, 500);
      });
      ros.on('error', function() {
          if(that.ros) that.ros.close();
          that.ros = undefined;
          setTimeout(that.connect, 500);
      });
    }

    this.init = function () {
      this.connect();
      
      var width = 800;
      var height = 600;
      // Create the main viewer.
      rosViewer = new ROS3D.Viewer({
        divID : canvasDiv,
        width : width,
        height : height,
        antialias : true,
        background : background,
        enableShadows: false,
        near: near,
        far: far,
        on_render: that.on_render,
        on_window_dblclick: function() {
          if(that.selectedMarker) {
              that.populate_query_select(libraryDiv,libSelectDiv);
              that.unselectMarker();
          }
        }
      });
      rosViewer.addObject(new ROS3D.Grid());
        
      var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : 'map'
      });

      this.setup_autocompletion();
      this.setup_history_field();
      this.setup_query_field();
      
      this.populate_query_select(libraryDiv,libSelectDiv);
      this.resize_canvas();
      
      set_inactive(document.getElementById(nextButtonDiv));
      
      var imageResizer = function(){
          var image = $('#mjpeg_image');
          var div = $('#'+pictureDiv);
          var image_width = that.imageWidth();
          var image_height = that.imageHeight();
          if(!image || image_width <= 0.0 || image_height <= 0.0) return true;
          
          var image_ratio = image_height/image_width;
          var div_ratio = div.height()/div.width();
          if(image_ratio < div_ratio) {
              image.width(div.width());
              image.height(div.width()*image_ratio);
          }
          else {
              image.height(div.height());
              image.width(div.height()/image_ratio);
          }
          return false;
      };
      $('#'+pictureDiv).resize(function(){
          var timeout = function(){
              if(imageResizer()) window.setTimeout(timeout, 10);
          };
          if(imageResizer()) window.setTimeout(timeout, 10);
      });
    };
    
    this.registerNodes = function () {
      if(isRegistered) return;
      isRegistered = true;
      
      // Setup publisher that sends a dummy message in order to keep alive the socket connection
      keepAlive = new KeepAlivePublisher({ros : ros, interval : 30000});
      
      // Setup a client to listen to TFs.
      var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/my_frame'
      });
        
      var pointCloud = new ROS3D.PointCloud2({
        ros : ros,
        topic : '/RoboSherlock/points',
        tfClient : tfClient,
        rootObject : rosViewer.scene,
        size: 0.02,
        max_pts: 50000
      });
        
      // Setup the marker client.
      var markerClient = new ROS3D.MarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/visualization_marker',
        sceneObjects : rosViewer.scene,
        selectableObjects : rosViewer.selectableObjects,
        backgroundObjects : rosViewer.backgroundScene
      });

      // Setup the marker array client.
      var markerArrayClient = new ROS3D.MarkerArrayClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/visualization_marker_array',
        rootObject : rosViewer.scene,        
        path : meshPath,
      });
      
//       var rsMarkerArrayClient = new ROS3D.MarkerArrayClient({
//         ros : ros,
//         tfClient : tfClient,
//         topic : '/RoboSherlock/markers',
//         rootObject : rosViewer.scene,        
//         path : meshPath,
//       });
     
      var activeAnnotatorListListener = new ROSLIB.Topic({
         ros: ros, 
         name: '/RoboSherlock/vis/active_annotators',
         messageType: 'robosherlock_msgs/RSActiveAnnotatorList'  
      });    
        
      activeAnnotatorListListener.subscribe( function(message) {
          document.getElementById(annotatorListDiv).innerHTML ="";
          var i;
          for (i = 0; i < message.annotators.length; i++) {
              newcontent = document.createElement('a');  
              newcontent.setAttribute("href",'#'+message.annotators[i]);
              newcontent.setAttribute("id",'myAnnotListElement');
              newcontent.setAttribute("onclick",'knowrob.setAnnotatorSelection()');
              newcontent.innerHTML=message.annotators[i];
              newcontent.onclick = function() {
                    document.getElementById('myInput').value = this.innerHTML;
                    document.getElementById('myDropdown').className='dropdown-content';
//                    $('#myInput').change();
              };
              document.getElementById(annotatorListDiv).appendChild(newcontent);
          }
          $('#'+annotatorListDiv).change();
      });  
        
      var desig_listener = new ROSLIB.Topic({
        ros : ros,
        name : '/RoboSherlock/result_advertiser',
        messageType : 'robosherlock_msgs/RSObjectDescriptions'
      });
        
      desig_listener.subscribe(function(message) {
        document.getElementById(designatorDiv).innerHTML = "";
        var i;
        for (i = 0; i < message.obj_descriptions.length; i++) { 
	        var obj = JSON.parse(message.obj_descriptions[i]);
            newcontent = document.createElement('p');    
            newContentID = "Objdescription"+i;
            newcontent.setAttribute("id",newContentID);
            newcontent.setAttribute("style","margin: 15px");

            newcontent2 = document.createElement('p');
            newcontent2.innerHTML = "Description of Obj#"+i;

            document.getElementById(designatorDiv).appendChild(newcontent2);   
            document.getElementById(designatorDiv).appendChild(newcontent);
 
            $('#'+newContentID).jsonViewer(obj,{collapsed: true}); 
        }
        $('#'+designatorDiv).change();
      });
      
      this.annotatorResultsControl = new ROSLIB.Service({
        ros : ros,
        name : '/RoboSherlock/vis_command',
        serviceType : 'robosherlock_msgs/RSVisControl'
      });
    
      
      this.waitForJsonProlog();
    };    
    
    this.waitForJsonProlog = function () {
        var client = new JsonProlog(ros, {});
        client.jsonQuery("true", function(result) {
            client.finishClient();
            
            if(result.error) {
                // Service /json_prolog/simple_query does not exist
                setTimeout(that.waitForJsonProlog, 500);
            }
            else {
                that.isPrologConnected = true;
      
                // Auto select the mongo database
                if(mngDBName) {
                    prolog = knowrob.new_pl_client();
                    prolog.jsonQuery("mng_db('"+mngDBName+"').", function(result) {
                        console.log("Selected mongo DB " + mngDBName);
                        prolog.finishClient();
                    });
                }
            }
        });
    };


    this.annotatorCommand = function(message) {
        var msgToSend = '';
        if (message == 'myInput'){
            this.msgToSend = document.getElementById('myInput').value; 
        }
        else {
            this.msgToSend = message;
        }   
        var request = new ROSLIB.ServiceRequest({
            command: this.msgToSend
            });
        console.log('Whaaa:' + request);
        this.annotatorResultsControl.callService(request, function(result) {
            document.getElementById('myInput').value = result.active_annotator;
            console.log('Result for service call on '+ result.success);
     });
        
        console.log('Sending request: '+this.msgToSend);
    };
    
    this.setup_history_field = function () {
        var history = ace.edit(historyDiv);
        history.setTheme("ace/theme/solarized_light");
        history.getSession().setMode("ace/mode/prolog");
        history.getSession().setUseWrapMode(true);
        history.setOptions({
            readOnly: true,
            showGutter: false,
            printMarginColumn: false,
            highlightActiveLine: false,
            highlightGutterLine: false
        });
        return history;
    };

    this.setup_query_field = function () {
        var userQuery = ace.edit(queryDiv);
        userQuery.resize(true);
        userQuery.setTheme("ace/theme/solarized_light");
        userQuery.getSession().setMode("ace/mode/prolog");
        userQuery.getSession().setUseWrapMode(true);
        userQuery.setOptions({
            showGutter: false,
            printMarginColumn: false,
            highlightActiveLine: false,
            highlightGutterLine: false,
            enableBasicAutocompletion: true
        });
        userQuery.commands.addCommand({
            name: 'send_query', readOnly: false,
            bindKey: {win: 'Enter',  mac: 'Enter'},
            exec: function(editor) { that.query(); }
        });
        userQuery.commands.addCommand({
            name: 'new_line', readOnly: false,
            bindKey: {win: 'Ctrl-Enter',  mac: 'Command-Enter'},
            exec: function(editor) { that.set_query_value(userQuery.getValue()+"\n"); }
        });
        userQuery.commands.addCommand({
            name: 'next_result', readOnly: false,
            bindKey: {win: 'Ctrl-;',  mac: 'Command-;'},
            exec: function(editor) { that.next_solution(); }
        });
        userQuery.commands.addCommand({
            name: 'next_result', readOnly: false,
            bindKey: {win: 'Ctrl-n',  mac: 'Command-n'},
            exec: function(editor) { that.next_solution(); }
        });
        userQuery.commands.addCommand({
            name: 'next_history', readOnly: false,
            bindKey: {win: 'Up',  mac: 'Up'},
            exec: function(editor) { that.set_next_history_item(); }
        });
        userQuery.commands.addCommand({
            name: 'previous_history', readOnly: false,
            bindKey: {win: 'Down',  mac: 'Down'},
            exec: function(editor) { that.set_previous_history_item(); }
        });
        
        // Create console iosOverlay
        var console = document.getElementById('console');
        if(console) {
            var consoleOverlay = document.createElement("div");
            consoleOverlay.setAttribute("id", "console-overlay");
            consoleOverlay.className = "ui-ios-overlay ios-overlay-hide div-overlay";
//            consoleOverlay.innerHTML += '<span class="title">Processing Query</span';
            consoleOverlay.style.display = 'none';
            console.appendChild(consoleOverlay);
        }
        
        // Create page iosOverlay
        var page = document.getElementById('page');
        if(page) {
            var pageOverlay = document.createElement("div");
            pageOverlay.setAttribute("id", "page-overlay");
            pageOverlay.className = "ui-ios-overlay ios-overlay-hide div-overlay";
            pageOverlay.innerHTML += '<span class="title">Please select an Episode</span';
            pageOverlay.style.display = 'none';
            page.appendChild(pageOverlay);
        }
        
        return userQuery;
    };
    
    this.setup_autocompletion = function() {
        // Add completer for prolog code
        ace.require("ace/ext/language_tools").addCompleter({
            getCompletions: function(editor, session, pos, prefix, callback) {
                var names = that.get_completions();
                if( names ) {
                  callback(null, names.map(function(x) {
                      return {name: x, value: x, score: 100, meta: "pl"};
                  }));
                }
            }
        });
    };
    
    this.loadTextureBinary = function ( data ) {
        var image = new Image();
        var texture = new THREE.Texture( image );
        image.onload = function () { texture.needsUpdate = true; };
        image.crossOrigin = this.crossOrigin;
        image.src = "data:image/png;base64," + data;
        //image.src = "data:image/png;base64," + Base64.encode(data);
        return texture;
    };
    
    this.new_pl_client = function() {
      if (prolog != null && prolog.finished == false) {
        ace.edit(historyDiv).setValue(ace.edit(historyDiv).getValue() + "stopped.\n", -1);
        ace.edit(historyDiv).navigateFileEnd();
        prolog.finishClient();
      }
      prolog = new JsonProlog(ros, {});
      return prolog;
    }
    
    this.get_completions = function() {
      if( ! prologNames ) {
        prolog = this.new_pl_client();
        prologNames = [];
        // Query for predicates/modules and collect all results
        prolog.jsonQuery("findall(X, current_predicate(X/_);current_module(X), L)", function(x) {
          if (x.value) {
            // Parse each value
            var lines = x.value.split("\n");
            for(i=1; i<lines.length-1; ++i) {
              var tmp = lines[i].split(" = ");
              if(tmp.length==2) {
                prologNames.push(tmp[1].trim());
              }
            }
            prologNames.sort();
          }
          else {
            console.warn("Unable to query prolog names.");
            console.warn(x);
          }
        }, mode=0);
      }
      return prologNames;
    };
    
    ///////////////////////////////
    //////////// Getter
    ///////////////////////////////
    
    this.get_ros = function () {
      return ros;
    };

    this.get_ros_viewer = function () {
      return rosViewer;
    };
    
    this.get_prolog_names = function() {
      return prologNames;
    };
    
    ///////////////////////////////
    //////////// Prolog queries
    ///////////////////////////////
    
    this.showConsoleOverlay = function () {
      var consoleOverlay = document.getElementById('console-overlay');
      if(consoleOverlay) {
          consoleOverlay.style.display = 'block';
          consoleOverlay.className = consoleOverlay.className.replace("hide","show");
          consoleOverlay.style.pointerEvents = "auto";
      }
    };
    this.hideConsoleOverlay = function () {
      var consoleOverlay = document.getElementById('console-overlay');
      if(consoleOverlay) {
          //consoleOverlay.style.display = 'none';
          consoleOverlay.className = consoleOverlay.className.replace("show","hide");
          consoleOverlay.style.pointerEvents = "none";
      }
    };
    
    var pageOverlayDisabled = false;
    this.showPageOverlay = function () {
      var pageOverlay = document.getElementById('page-overlay');
      if(pageOverlay && !pageOverlayDisabled) {
          pageOverlay.style.display = 'block';
          pageOverlay.className = pageOverlay.className.replace("hide","show");
          pageOverlay.style.pointerEvents = "auto";
      }
    };
    this.hidePageOverlay = function () {
      var pageOverlay = document.getElementById('page-overlay');
      if(pageOverlay && !pageOverlayDisabled) {
          //pageOverlay.style.display = 'none';
          pageOverlay.className = pageOverlay.className.replace("show","hide");
          pageOverlay.style.pointerEvents = "none";
      }
    };
    this.disablePageOverlay = function () {
      pageOverlayDisabled = true;
    };
    
    this.query = function () {
      var query = ace.edit(queryDiv);
      var history = ace.edit(historyDiv);
      var q = query.getValue().trim();
    
      if (q.substr(q.length - 1) == ".") {
        q = q.substr(0, q.length - 1);
        prolog = this.new_pl_client();
        that.showConsoleOverlay();
        
        history.setValue(history.getValue() + "\n\n?- " + q +  ".\n", -1);
        history.navigateFileEnd();
        set_active(document.getElementById(nextButtonDiv));
        
        prolog.jsonQuery(q, function(result) {
            that.hideConsoleOverlay();
            history.setValue(history.getValue() + prolog.format(result), -1);
            history.navigateFileEnd();
            if( ! result.value ) set_inactive(document.getElementById(nextButtonDiv));
        }, mode=1); // incremental mode
        
        query.setValue("");
        
        this.add_history_item(q);
        historyIndex = -1;
      }
      else {
        if (prolog != null && prolog.finished == false) {
          that.hideConsoleOverlay();
          history.setValue(history.getValue() + "stopped.\n\n", -1);
          history.navigateFileEnd();
          prolog.finishClient();
        }
        else {
          alert("Invalid prolog query '" + q + "'. Prolog queries always end with a dot.");
        }
      }
    };

    this.next_solution = function () {
      var history = ace.edit(historyDiv);
      that.showConsoleOverlay();
      prolog.nextQuery(function(result) {
          that.hideConsoleOverlay();
          history.setValue(history.getValue() + prolog.format(result), -1);
          history.navigateFileEnd();
          if( ! result.value ) set_inactive(document.getElementById(nextButtonDiv));
      });
      user_query.focus();
    };
    
    function set_active(div) {
      div.style.pointerEvents = "auto";
      div.style.backgroundColor = "#dadada";
      div.style.color = "#606060";
    };
    
    function set_inactive(div) {
      div.style.pointerEvents = "none";
      div.style.backgroundColor = "#cfcfcf";
      div.style.color = "#adadad";
    };

    // append the selected query to the user_query form
    this.add_selected_to_queryform = function (selectid, focus) {
      var select = document.getElementById(selectid);
      this.set_query_value(select.options[select.selectedIndex].value, focus);
    };

    // set the value of the query editor and move the cursor to the end
    this.set_query_value = function (val, focus){
      var user_query = ace.edit(queryDiv);
      user_query.setValue(val, -1);
      if(focus) user_query.focus();
      user_query.navigateFileEnd();
    };

    
    ///////////////////////////////
    //////////// History
    ///////////////////////////////

    this.add_history_item = function (query) {
        $.ajax({
            url: "/robosherlock/add_new_query",
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({query: query}),  
            dataType: "json"
        }).done( function (request) {})
    };
    this.set_history_item = function (index) {
        $.ajax({
            url: "/robosherlock/get_history_query",
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({index: index}),  
            dataType: "json",
            success: function (data) {
                 ace.edit(queryDiv).setValue(data.item);
                 historyIndex = data.index;
            }
        }).done( function (request) {})
    };

    this.set_next_history_item = function () {
        this.set_history_item(historyIndex+1);
    };
    
    this.set_previous_history_item = function () {
        this.set_history_item(historyIndex-1);
    };

    this.resize_canvas = function () {
        that.resize_canvas_($('#'+canvasDiv).width(), $('#'+canvasDiv).height());
    }
    
    this.resize_canvas_ = function (w,h) {
      rosViewer.renderer.setSize(w, h);
      rosViewer.camera.aspect = w/h;
      rosViewer.camera.updateProjectionMatrix();
      
      rosViewer.camera.left = - w / 2;
      rosViewer.camera.right = w / 2;
      rosViewer.camera.top = h / 2;
      rosViewer.camera.bottom = - h / 2;
      rosViewer.camera.updateProjectionMatrix();
    };
    
    this.get_episode_data = function (idx) {
//         if(!that.episodeData) {
            try {
                // url must point to a json-file containing an array named "query" with
                // the query strings to display in the select
                // FIXME: bad synchron request
                var request = new XMLHttpRequest
                request.open("GET", episodeURL[idx], false);
		request.overrideMimeType("application/json");
                request.send(null);
                that.episodeData = JSON.parse(request.responseText);
            }
            catch(e) {
                console.warn("Failed to load episode data.");
            }
//         }
        return that.episodeData;
    };

    // fill the select with json data from url
    this.populate_query_select = function (id,isSel,queries) {
//	idx = document.getElementById(isSel).selectedIndex;
        if(queries == undefined) {
            var episodeData = that.get_episode_data(0);
            if(!episodeData) return;
            queries = episodeData.query;
        }
        
        var select = document.getElementById(id);
        if(select !== null) {
          while (select.firstChild) select.removeChild(select.firstChild);
          
          for (var i = 0; i < queries.length; i++) {
            var opt = document.createElement('option');
            if(queries[i].q !== undefined) {
              opt.value = queries[i].q;
              opt.innerHTML = queries[i].text;
              select.appendChild(opt);
            }
          }
          select.size = queries.length;
        }
    };

    this.highlight_element = function (name, type, highlight) {
      if(typeof type == "string") {
        if(type === "class") {
          var elems = document.getElementsByClassName(name);
          for (i = 0; i < elems.length; i++) {
            elems[i].style.backgroundColor = highlight ? "#144F78" : "#BBB";
            elems[i].style.border = highlight ? "5px solid #144F78" : "1px solid #BBB";
          }
        }
        else if(type === "id") {
          document.getElementById(name).style.border = highlight ? "5px solid #144F78" : "1px solid #BBB";
        }
      }
    }; 

    // hook for links of class "show_code" that pastes the content of the
    // previous code block into the query field
    $( document ).ready(function() {
      $( "a.show_code" ).click(function( event ) {
        that.set_query_value( $(this).closest("pre + *").prev().find('code').html() );
        event.preventDefault();
      });
    });
}
