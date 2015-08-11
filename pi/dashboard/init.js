timer = null;

function on_load(){
  $(".move").click(function(){remote.doMove(this.id)});
  $(".tool").click(function(){ui.tool = this.id});
  $(".drone-status").click(function(){remote.doGoStatus(this.id)});
  $("#reset_button").click(remote.doReset);
  $("#remote").val(remote.url);
  $("#settings_save").click(function(){
    remote.url = $("#remote").val();
    $("#settings").modal("hide");
  });
  $("#connect").click(function(){
    if($(this).hasClass("btn-primary")){
        stat_start();
        $(this).text("Disconnect");
    }else{
        stat_pause();
        $(this).text("connect");
    }
    $(this).toggleClass("btn-primary");
    $(this).toggleClass("btn-success");
  });
  //
  map.init("allmap");
  map.moveMark(map.transToPoint([map.homex, map.homey]), 'station', 'station');

}


function stat_start(){
  var rep = function(){
    remote.getAll(
      function(stat){
        $("#altimeter").text('Altimeter: '+stat.altimeter);
        ui.setProgressBar("pbar", Number(stat.battery));
        $("#gps").text('GPS: '+stat.gps);
        $("#pose").text('Pose: '+stat.pose);
        var p = map.transToPoint(stat.gps);
        map.gotoPos(p);
        map.moveMark(p, 'drone', 'drone');
        ui.setDroneStatus(stat.status);
      });
    // remote.getAltimeter(
    //   function(n){$("#altimeter").text('Altimeter: '+n);}
    // );
    // remote.getGPS(function(s){
    //   $("#gps").text('GPS: '+s);
    //   var p = map.transToPoint(s);
    //   map.gotoPos(p);
    //   map.moveMark(p, 'drone', 'drone');
    // });
    $("#front_cam").attr("src",remote.getCamURL("front"));
    $("#station_cam").attr("src",remote.getCamURL("station"));
  };
  timer = setInterval(rep, 500);
}

function stat_pause(){
  clearInterval(timer);
  ui.resetDroneStatus();
  map.deleteMark("drone");
  map.deleteMark("goal");
  $("#front_cam").attr("src", "static/placehold.jpg");
  $("#station_cam").attr("src", "static/placehold.jpg");
}
