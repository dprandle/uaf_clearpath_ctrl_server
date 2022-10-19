var express = require('express');
var app = express();
const path = require('path');

app.get('/', function(req, res){
   res.sendFile(path.join(__dirname, 'emscripten', 'jackal_ctrl.html'));
});

app.use(express.static(path.join(__dirname, 'emscripten')));

app.listen(80);