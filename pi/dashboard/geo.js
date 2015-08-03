var map = {
  map: null,
  marks: {},
  init: function(id){
    this.map = new BMap.Map(id);
    this.map.centerAndZoom(new BMap.Point(113.659261, 34.799606), 16);
    this.map.addControl(new BMap.MapTypeControl());
    this.map.enableScrollWheelZoom(true);
  },
  gotoPos: function(p){
    this.map.centerAndZoom(p, 16);
  },

  moveMark: function(name, p){
    if(this.marks[name])
      this.map.removeOverlay(this.marks[name]);
    this.marks[name] = new BMap.Marker(p);
    this.map.addOverlay(this.marks[name]);
  },

  transToPoint: function(str){
    var pos = JSON.parse(str);
    return new BMap.Point(pos[0], pos[1]);
  }
};
