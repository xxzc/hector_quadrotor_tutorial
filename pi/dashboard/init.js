timer = null;

function on_load(){
  $(".move").click(function(){remote.doMove(this.id)});
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
}


function stat_start(){
  var rep = function(){
    remote.getAltimeter(
      function(n){$("#altimeter").text('Altimeter: '+n);}
    );
    remote.getGPS(function(s){
      $("#gps").text('GPS: '+s);
      var p = map.transToPoint(s);
      map.gotoPos(p);
      map.moveMark('drone', p);
    });
    $("#front_cam").attr("src",remote.getCamURL("front"));
    $("#station_cam").attr("src",remote.getCamURL("station"));
  };
  timer = setInterval(rep, 500);
}

function stat_pause(){
  clearInterval(timer);
}
