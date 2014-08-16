function extend(Child, Parent){
	var F = function(){};
	F.prototype = Parent.prototype;
	Child.prototype = new F();
	Child.prototype.constructor = Child;
	Child.uber = Parent.prototype;
}

function randomVector(vec1, vec2){
	var rez = new APE.Vector3();
	rez.x = vec1.x + (vec2.x - vec1.x) * Math.random();
	rez.y = vec1.y + (vec2.y - vec1.y) * Math.random();
	rez.z = vec1.z + (vec2.z - vec1.z) * Math.random();
	return rez;
}

/**
 * Firework mesh definition and colour set up.
 */
 
var size = 0.1, detail = 5;
var shape = new THREE.SphereGeometry(size, detail, detail);
var materials = [
	new THREE.MeshBasicMaterial({color:0xff0000}),
	new THREE.MeshBasicMaterial({color:0xff8000}),
	new THREE.MeshBasicMaterial({color:0xffff00}),
	new THREE.MeshBasicMaterial({color:0x00ff00}),
	new THREE.MeshBasicMaterial({color:0x00ffff}),
	new THREE.MeshBasicMaterial({color:0x6666ff}),
	new THREE.MeshBasicMaterial({color:0xff00ff}),
	new THREE.MeshBasicMaterial({color:0xffffff}),
	new THREE.MeshBasicMaterial({color:0xff8080})
];

/**
  * Fireworks are particles, with additional data for rendering and
  * evolution
  */
  
function Firework(){
	/** Fireworks inherit the properties of the particle class*/
	APE.Particle.apply(this, arguments);
	
	/** Fireworks have an integer type, used for firework rules.*/ 
	this.type = 0;
	
	/**
	  * The age of a firework determines when it detonates. Age gradually
	  * decreases; when it passes zero the firework delivers its payload.
	  * Think of age as fuse left.
	  */
	this.age = 0;
	
	/** Create mesh for rendering. */
	this.mesh = new THREE.Mesh(shape, materials[1]);
	this.mesh.visible = false;
	scene.add(this.mesh);
}
/** Inherit the prototype chain from Particle class. */
extend(Firework, APE.Particle);

/** Updates the firework by the given duration of time. Returns true
  * if the firework has reached the end of its life and needs to be
  * removed.
  */
Firework.prototype.update = function (duration) { 
	// Update our physical state
	this.integrate(duration);
	
	// We work backward from our age to zero
	this.age -= duration;
	return ((this.age < 0) || (this.position.y <0));
};

/** 
  * The payload is the new firework type to create when this
  * firework's fuse is over.
  */
function Payload(){
	/** The type of new particles to create. */
	this.type = 0;
	
	/** The number of particles in this payload. */
	this.count = 0;
}
/** Sets the payload properties in one go.*/
Payload.prototype.set = function(type, count){
	this.type = type;
	this.count = count;
};


/**
  * Firework rules control the length of a firework's fuse and the 
  * particles it should evolve into.
  */
  
function FireworkRule(){
	/** The type of firework that is managed by this rule. */
	this.type = 0;
	
	/** The minimum length of the fuse. */
	this.minAge = 0;
	
	/** The maximum length of the fuse. */
	this.maxAge = 0;
	
	/** The minimum relative velocity of this firework. */
	this.minVelocity = new APE.Vector3();
	
	/** The maximum relative velocity  of this firework. */
	this.maxVelocity = new APE.Vector3();
	
	/** The damping of this firework type. */
	this.damping = 1;
	
	/** The number of payloads for this firework type. */
	this.payloadCount = 0;
	
	/** The set of payload. */
	this.payloads = null;
}

/** the set of payloads. */
FireworkRule.prototype.init = function  (payloadCount) {
	this.payloadCount = payloadCount;
	this.payloads = [];
	for(var i = 0; i < this.payloadCount; i++){
		this.payloads.push(new Payload());
	}
};
/**
  * Set all the rule parameters in one go.
  */

FireworkRule.prototype.setParameters = function (type, minAge, maxAge, minVelocity, maxVelocity, damping ) {
	this.type = type;
	this.minAge = minAge;
	this.maxAge = maxAge;
	this.minVelocity =  minVelocity;
	this.maxVelocity = maxVelocity;
	this.damping = damping;
};

/**
  * Creates a new firework of this type and writes it into the given
  * instance. The optional parent firework is used to base position 
  * and velocity on.
  */
FireworkRule.prototype.create = function (firework, parent){
	firework.type = this.type;
	firework.age = this.minAge + (this.maxAge - this.minAge) * Math.random();
	
	var vel = new APE.Vector3();
	if(parent){
		// the position and velocity are based on the parent.
		firework.setPosition(parent.getPosition());
		vel = vel.add(parent.getVelocity());
	}else{
		var start = new APE.Vector3();
		var x = Math.floor(Math.random() * 3);
		start.x = -5.0 + x * 5.0;
		firework.setPosition(start);
	}
	vel = vel.add(randomVector(this.minVelocity, this.maxVelocity));
	firework.setVelocity(vel);
	
	// We use a mass of one in all cases (no point having fireworks
	// with different masses, since they are only under the influence 
	// of gravity).
	
	firework.setMass(1);
	
	firework.setDamping(this.damping);
	
	firework.setAcceleration(APE.GRAVITY);
	
};

/**
 * The main demo class definition.
 */
 
 function FireworksDemo(){
	/**
	 * Holds the maximum number of fireworks that can be in use.
	 */
	this.maxFireworks = 1024;
	
	/** Holds the firework data. */
	this.fireworks = [];
	
	/** Holds the index of the next firework slot to use. */
	this.nextFirework = 0;
	
	/** And the number of rules. */
	this.ruleCount = 11;
	
	/** Holds the set of rules. */
	this.rules = [];
	
	//Make all shots unused, initialize rules
	for(var i = 0; i< this.maxFireworks; i++){
		this.fireworks.push(new Firework());
	}
	
	for(var i = 0; i< this.ruleCount; i++){
		this.rules.push(new FireworkRule());
	}
	
	// Create the firework type;
	this.initFireWorkRules();
	
 }
 

 FireworksDemo.prototype.initFireWorkRules = function () {
	// Go through the firework  types and create their rules.
	this.rules[0].init(2);
	this.rules[0].setParameters(
		1, // type
		0.5, 1.4, // age range
		new APE.Vector3(-5, 25, -5), // minVelocity
		new APE.Vector3(5, 28, 5), // maxVelocity
		0.1// damping
	);
	this.rules[0].payloads[0].set(3, 5);
	this.rules[0].payloads[1].set(5, 5);

	this.rules[1].init(1);
	this.rules[1].setParameters(
		2, // type
		0.5, 1.0, // age range
		new APE.Vector3(-5, 10, -5), // minVelocity
		new APE.Vector3(5, 20, 5), // maxVelocity
		0.8// damping
	);
	this.rules[1].payloads[0].set(4, 2);
	
	this.rules[2].init(0);
	this.rules[2].setParameters(
		3, // type
		0.5, 1.5, // age range
		new APE.Vector3(-5, -5, -5), // minVelocity
		new APE.Vector3(5, 5, 5), // maxVelociy
		0.1// damping
	);
	
	this.rules[3].init(0);
	this.rules[3].setParameters(
		4, // type
		0.25, 5, // age range
		new APE.Vector3(-20, 5, -5), // minVelocity
		new APE.Vector3(20, 5, 5), // maxVelociy
		0.2// damping
	);

	this.rules[4].init(1);
	this.rules[4].setParameters(
		5, // type
		0.5, 1.0, // age range
		new APE.Vector3(-20, 2, -5), // minVelocity
		new APE.Vector3(20, 18, 5), // mxaxVelociy
		0.1// damping
	);
	this.rules[4].payloads[0].set(3, 5);
	
	this.rules[5].init(0);
	this.rules[5].setParameters(
		6, // type
		3, 5, // age range
		new APE.Vector3(-5, 5, -5), // minVelocity
		new APE.Vector3(5, 10, 5), // mxaxVelociy
		0.95// damping
	);
	
	this.rules[6].init(1);
	this.rules[6].setParameters(
		7, // type
		4, 5, // age range
		new APE.Vector3(-5, 50, -5), // minVelocity
		new APE.Vector3(5, 60, 5), // mxaxVelociy
		0.01// damping
	);
	this.rules[6].payloads[0].set(8, 10);
	
	this.rules[7].init(0);
	this.rules[7].setParameters(
		8, // type
		0.25, 0.5, // age range
		new APE.Vector3(-1, 1, -1), // minVelocity
		new APE.Vector3(1, 1, 1), // mxaxVelociy
		0.01// damping
	);
	
	this.rules[8].init(0);
	this.rules[8].setParameters(
		9, // type
		3, 5, // age range
		new APE.Vector3(-15, 10, -5), // minVelocity
		new APE.Vector3(15, 15, 5), // mxaxVelociy
		0.95// damping
	);
	
	// Initial projectile
	this.rules[9].init(1);
	this.rules[9].setParameters(
		10, // type
		0.5, 1.4, // age range
		new APE.Vector3(-5, 25, -5), // minVelocity
		new APE.Vector3(5, 28, 5), // mxaxVelociy
		0.1// damping
	);
	this.rules[9].payloads[0].set(3,5);
	
};
 
 
 FireworksDemo.prototype.create = function(){
	 /** Dispatches a firework from the origin. */
	if(arguments.length < 3){
		var type = arguments[0];
		var parent = arguments[1];
		
		// Get the rule needed to create this firework
		var rule = this.rules[type - 1];
		
		//create the firework
		rule.create(this.fireworks[this.nextFirework], parent);
		// Increment the index for the next firework
		this.nextFirework = (this.nextFirework + 1) % this.maxFireworks;
		
	}else{
		var type = arguments[0];
		var number = arguments[1];
		var parent = arguments[2];
				
		for(var i = 0; i < number; i++){
			this.create(type, parent);
		}
	}
 };
 
 
 FireworksDemo.prototype.update = function(){
	// Find the duration of the last frame in seconds;
	var duration = 1/60;
	if(duration <= 0.0) {
		return;
	}
	var firework;
	for(var i = 0; i < this.maxFireworks; i++) {
		firework = this.fireworks[i];
		// check if we need to process this firework
		if(firework.type > 0) {
			//Does it need removing?
			if(firework.update(duration)){
				//Find the appropriate rule
				var rule  = this.rules[firework.type - 1];

				// Delete the current firework (this doesn't affect its
				// position and velocity for passing to the create function
				// just whether or not it is processed for rendering or 
				// physics.)
				
				firework.type = 0;
				
				// Add the payload
				for (var j = 0; j < rule.payloadCount; j++){
					var payload = rule.payloads[j];
					this.create(payload.type, payload.count, firework);
				}
			
			}
		}
	}
	
};
 
FireworksDemo.prototype.display = function(){
	for(var i = 0; i < this.maxFireworks; i++){
		var firework = this.fireworks[i];
		if(firework.type > 0){
			firework.mesh.material = materials[firework.type-1];
			firework.mesh.visible = true;
			
			firework.mesh.position.x = firework.position.x;
			firework.mesh.position.y = firework.position.y;
			firework.mesh.position.y = firework.position.y;
		}else{
			firework.mesh.visible = false; 
		}
	}
};
 
FireworksDemo.prototype.key = function(key){
	switch(key){
		case '1': 
			this.create(1, 1, undefined);
			break;
		case '2': 
			this.create(2, 1, undefined);
			break;
		case '3': 
			this.create(3, 1, undefined);
			break;
		case '4': 
			this.create(4, 1, undefined);
			break;
		case '5': 
			this.create(5, 1, undefined);
			break;
		case '6': 
			this.create(6, 1, undefined);
			break;
		case '7': 
			this.create(7, 1, undefined);
			break;
		case '8': 
			this.create(8, 1, undefined);
			break;
		case '9': 
			this.create(9, 1, undefined);
			break;
	}
};
 