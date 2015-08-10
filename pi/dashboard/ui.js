var ui = {
  tool: "thand",
  setDroneStatus: function(sname){
    ui.resetDroneStatus();
    ui.setButton(sname, "btn-primary");
  },
  resetDroneStatus: function(){
    ui.setButton("ready", "btn-default");
    ui.setButton("landing", "btn-default");
    ui.setButton("nav", "btn-default");
  },
  setButton: function(id, cls){
    $('#'+id).removeClass('btn-default');
    $('#'+id).removeClass('btn-primary');
    $('#'+id).addClass(cls);
    // document.getElementById(id).className = "btn " + cls;
  }
}

var map = {
  map: null,
  marks: {},
  icons: {},
  homex: 113.659261, homey: 34.799606,

  init: function(id){
    this.map = new BMap.Map(id);
    this.map.centerAndZoom(new BMap.Point(this.homex, this.homey), 16);
    this.map.addControl(new BMap.MapTypeControl());
    this.map.enableScrollWheelZoom(true);
    this.icons.drone = new BMap.Icon("static/drone.png", new BMap.Size(30,30));
    this.icons.station = new BMap.Icon("static/station.png", new BMap.Size(30,30));
    this.icons.goal = new BMap.Icon("static/goal.png", new BMap.Size(30,30));
    this.map.addEventListener("click", function(e){
      switch (ui.tool) {
        case "tgoal":
          var gx = e.point.lng;
          var gy = e.point.lat;
          // alert(e.point.lng + ", " + e.point.lat);
          map.moveMark(map.transToPoint([gx, gy]), "goal", "goal");
          remote.doGoGoal(gx, gy);
          break;
        default:

      }

      });
  },
  onClick: function(f){
    this.map.addEventListener("click", f);
  },
  gotoPos: function(p){
    this.map.centerAndZoom(p, 16);
  },

  moveMark: function(p, name, ico){
    if(this.marks[name])
      this.map.removeOverlay(this.marks[name]);
    this.marks[name] = new BMap.Marker(p,{icon:this.icons[ico]});
    // this.marks[name].enableDragging();
    // this.marks[name].addEventListener("click", function(e){
    //
    // });
    this.map.addOverlay(this.marks[name]);
  },

  deleteMark: function(name){
    this.map.removeOverlay(this.marks[name]);
  },

  transToPoint: function(pos){
    return new BMap.Point(pos[0], pos[1]);
  }
};
