/* Js file for all the shit the query answering interface might need*/

//this name might just need changing
function RoboSherlock(options){

    var that = this;
    var activeDB = "PnP09ObjSymbolicGTFixed";
    var changedDB = true;
    var itemNo = 3;
    var lastQueryType = "None";
    var historyDiv = options.history_div || 'history';
    var queryDiv = options.query_div || 'user_query';
    var libraryDiv = options.library_div ||'querylist';
    var contentDiv = options.conent_div || 'bodyDiv';
    var noOfItems = 0;
    var queriesData = undefined;
    var queries =undefined;
    var noOfReq=0;
    var items = true;
    this.init = function () {
        this.setup_history_field();
        this.setup_query_field();
        this.get_query_data();
    }
    $("#db_names").click(function () {
        var item = $(this);
        if (item.val() != activeDB){
            activeDB = item.val();
            changedDB = true;
            $.ajax({
                type: "POST",
                url: "/change_db",
                async: true,
                data: activeDB,
                beforeSend: function(xhr){xhr.setRequestHeader('Content-type', 'text-plain');},
                success: function(data) {
                    $("#from_time_hypo").html(data);
                    $("#to_time_hypo").html(data);
                    $("#from_time").html(data);
                    $("#to_time").html(data);
                }
            });
        }
    });
    $("#btn_query").click(function (){
        that.query()
    });
    $("#export_button").click(function () {

        if (lastQueryType != "None"){
              window.location = '/export_data/' + new Date().getTime();
        }
    });
    $(document).on('click', '.page_collapsible', function () {
        var elem = $(this);
        var my_css  = {display: 'none'};
        if (elem.hasClass("collapse-close")){
            elem.removeClass("collapse-close");
            elem.addClass("collapse-open");
            my_css = {display: 'block'}
            var halo = elem.next('.container');
            halo.css(my_css)
        }else{
            elem.removeClass("collapse-open");
            elem.addClass("collapse-close");
            var halo = elem.next('.container');
            halo.css(my_css)
        }
    });
    $("#filter_button").click(function () {
        items = true;
        noOfReq = 0;
        that.form_query();
    });
    this.getMOreData = function(contentTab){
        $.ajax({
               type: "POST",
               url: "/get_more_data",
               async: true,
               data : contentTab,
               beforeSend: function(xhr){xhr.setRequestHeader('Content-type', 'text-plain');},
               success: function(data){
                   if (data != "NU"){
                       $("#maintable>tbody").append(data);
                        noOfReq--;
                    }else{
                       items = false;
                   }
                   }
            });
    };
    $("#bodyDiv").scroll(function () {
        if (noOfReq < 1 && items){
        var thisDiv = $(this);
        var scrollTop = thisDiv.scrollTop();
        var height = thisDiv[0].scrollHeight - thisDiv[0].offsetHeight;
        if (scrollTop >= height * 0.7){
            var active_tab = $("#query_tabs").find('li.active');
            var content_tab = active_tab.attr("id");
            noOfReq++;
            that.getMOreData(content_tab);
        }
        }
    });

    this.form_query_scenes = function () {
        lastQueryType = "export_scenes";
        var query = that.sceneJSON();
        $.ajax({
           type: "POST",
           url: "/get_scenes",
           data: JSON.stringify({scene: query}),
           async: true,
           beforeSend: function(xhr){xhr.setRequestHeader('Content-type', 'text-plain');},
           success: function(data){
                $("#bodyDiv").html(data);
           }
         });
    };

    this.sceneJSON = function () {
        var sceneQuery = {};
        if ($("#timestamp_check").is(":checked")) {
            sceneQuery['timestamp'] = {gt: $("#from_time").val(), lt: $("#to_time").val()}
        }
        if($("#objects_check").is(":checked")){
            sceneQuery['obj'] = {ids: $("#objects_id").val().split(",").map(function (item) {
                    return parseInt(item, 10);
                })};
        }
        return sceneQuery;
    };

    this.form_query_hypothesis = function () {
        lastQueryType = "export_hypothesis";
        var hypo = that.hyposJson()
        $.ajax({
           type: "POST",
           url: "/get_hypothesis",
           async: true,
           data: JSON.stringify({hypothesis: hypo}),
           beforeSend: function(xhr){xhr.setRequestHeader('Content-type', 'text-plain');},
           success : function (data) {
                $("#bodyDiv").html(data);
           } 
        });
    }
    this.form_query_objects = function () {
        lastQueryType = "export_objects";
        $.ajax({
           type: 'POST',
           url: '/get_objects',
           async: true,
           beforeSend: function(xhr){xhr.setRequestHeader('Content-type', 'text-plain');},
           success : function (data) {
                $("#bodyDiv").html(data);
           }
        });
    }

    this.hyposJson = function(){
        var hypoQuery = {}
        if($("#shape_check").is(":checked")){
            var shapeVal = $("#shapes_hypo").val();
            var shapeConf = parseFloat($("#shape_conf").val() + "0.0");
            hypoQuery['shape'] = {val: shapeVal, conf: shapeConf}
        }
        if ($("#size_check").is(":checked")){
            var sizeVal = $("#size_hypo").val();
            var sizeConf = parseFloat($("#size_conf").val() + "0.0");
            hypoQuery['size'] = {val: sizeVal, conf: sizeConf}
        }
        if ($("#class_check").is(":checked")){
            var classVal = $("#class_val").val();
            var classConf = parseFloat($("#class_conf").val() + "0.0");
            hypoQuery['class'] = {val: classVal, conf: classConf}
        }
        if ($("#timestamp_check_hypo").is(":checked")) {
            hypoQuery['timestamp'] = {gt: $("#from_time_hypo").val(), lt: $("#to_time_hypo").val()}
        }
        if($("#obj_check_hypo").is(":checked")){
            hypoQuery['obj'] = {id: $("#obj_hypo").val()}
        }
        return hypoQuery;
    }



    this.form_query = function()
    {
        var active_tab = $("#query_tabs").find('li.active');
        var content_tab = active_tab.attr("id");
        switch (content_tab) {
            case "scenes_tab" :
                that.form_query_scenes();
                break;
            case "hypothesis_tab" :
                that.form_query_hypothesis();
                break;
            case "objects_tab" :
                that.form_query_objects();
                break;
            case "query_tab" :
                that.query();
                break;
        }
    };

    this.query = function()
    {
       var user_query = ace.edit(queryDiv);
       var history = ace.edit(historyDiv);

        var q = user_query.getValue().trim();
        history.renderer.$cursorLayer.element.style.display = "none"
        history.setValue(history.getValue() + "\n\n?- " + q +  "\n", -1);
        history.navigateFileEnd();

        $.ajax({
           type: "POST",
           url: "/prolog_query",
           data: q, // serializes the form's elements.
           async: true,
           beforeSend: function(xhr){xhr.setRequestHeader('Content-type', 'text-plain');},
           success: function(data){
                $("#bodyDiv").html(data);
                // $("#bodyDiv-div").find("script").each(function(i) {
                //     eval($(this).text());
                // });
           }
         });
        user_query.setValue("")
    }


    this.get_query_data = function () {
        if(!that.queryData) {
            try {
                var xmlHttp = new XMLHttpRequest();
                xmlHttp.open("GET", "/_get_queries", true); // true for asynchronous
                xmlHttp.onload = function(e) {
                    if (xmlHttp.readyState == 4 && xmlHttp.status == 200)
                    {
                        that.queryData = JSON.parse(xmlHttp.responseText);
                        that.populate_query_select(libraryDiv, that.queryData);
                    }
                }
                xmlHttp.send(null);
            }
            catch(e) {
                console.warn("Failed to load episode data.");
            }
        }
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
            pageOverlay.innerHTML += '<span class="title">Please select an Episode</span>';
            pageOverlay.style.display = 'none';
            page.appendChild(pageOverlay);
        }

        return userQuery;
    };

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


    // fill the select with json data from url
    this.populate_query_select = function (id, queryData) {
        queries = queryData.query;
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
}