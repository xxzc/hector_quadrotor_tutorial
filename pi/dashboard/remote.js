
var remote = {
  url: 'http://127.0.0.1:5000',
  getAltimeter: function(done){
    $.ajax(this.url + '/data/altimeter').done(done);
  },
  getGPS: function(done){
    $.ajax(this.url + '/data/gps').done(done);
  },
  getCamURL: function(cam){
    return this.url + "/data/cam/" + cam + "?" + new Date().getTime();
  },
  doMove: function(move){
    $.ajax({'url': this.url + "/move/" + move});
  },
  doReset: function(){
    $.ajax({'url': this.url + "/action/reset"});
  }
};
