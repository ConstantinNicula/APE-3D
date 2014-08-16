/**
  * Creates the basic context for the APE-3D engine, a lightweight, impulse-based,
  * mass aggregate physics engine. APE-3D stands for Another Physics Engine.    
  */

var APE = { VERSION : 'Alpha-2' };

APE.extend = function(child, parent){
    function F(){}
    F.prototype = parent.prototype;
    child.prototype =  new F();
    child.prototype.constructor = child;
};
