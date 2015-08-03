remote = 'http://127.0.0.1:5000';
timer = null;
function on_load(){
  $(".move").click(function(){$.ajax({url: remote + "/move/" + this.id});});
  $("#reset_button").click(function(){$.ajax({url: remote + "/action/reset"});});
  $("#remote").val(remote);
  $("#settings_save").click(function(){
    remote = $("#remote").val();
    $("#settings").modal("hide");
  });
  $("#connect").click(function(){
    if($(this).hasClass("btn-primary")){
        stat_start();
        $(this).text("Connect");
    }else{
        stat_pause();
        $(this).text("Disconnect");
    }
    $(this).toggleClass("btn-primary");
    $(this).toggleClass("btn-success");
  });
  //
  map_init();
}

function map_init(){
  var map = new BMap.Map("allmap");
  map.centerAndZoom(new BMap.Point(116.404, 39.915), 11);
  map.addControl(new BMap.MapTypeControl());
  map.setCurrentCity("beijing");
  map.enableScrollWheelZoom(true);
}

function stat_start(){
  var rep = function(){
    $.ajax(remote + '/data/altimeter').done(function(n){$("#altimeter").text('Altimeter:'+n);});
    $("#front_cam").attr("src",remote + "/data/cam/front?" + new Date().getTime());
    $("#station_cam").attr("src",remote + "/data/cam/station?" + new Date().getTime());
  };
  timer = setInterval(rep, 500);
}

function stat_pause(){
  clearInterval(timer);
}
