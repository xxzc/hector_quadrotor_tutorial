var map = {
  map: null,
  marks: {},
  icons: {},
  init: function(id){
    this.map = new BMap.Map(id);
    this.map.centerAndZoom(new BMap.Point(113.659261, 34.799606), 16);
    this.map.addControl(new BMap.MapTypeControl());
    this.map.enableScrollWheelZoom(true);
    this.icons.drone = new BMap.Icon("static/drone.png", new BMap.Size(30,30));
    this.icons.station = new BMap.Icon("static/station.png", new BMap.Size(30,30));
  },
  gotoPos: function(p){
    this.map.centerAndZoom(p, 16);
  },

  moveMark: function(p, name, ico){
    if(this.marks[name])
      this.map.removeOverlay(this.marks[name]);
    this.marks[name] = new BMap.Marker(p,{icon:this.icons[ico]});
    this.map.addOverlay(this.marks[name]);
  },

  transToPoint: function(str){
    var pos = JSON.parse(str);
    return new BMap.Point(pos[0], pos[1]);
  }
};
