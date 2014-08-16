// simulate enum type
var UNUSED = 0,
	PISTOL = 1,
	ARTILLERY = 2,
	FIREBALL = 3,
	LASER = 4;


function AmmoRound(){
	this.particle = new APE.Particle();
	this.type = 0;
	this.startTime = 0;
	this.mesh = new THREE.Mesh(
		new THREE.SphereGeometry(0.2, 5, 5),
		new THREE.MeshBasicMaterial({color:0x000000})
	);
	this.mesh.visible = false;
	scene.add(this.mesh);
}
AmmoRound.prototype.render = function(){
	if(this.type != UNUSED){
		var pos = this.particle.getPosition();
		this.mesh.position.set(pos.x, pos.y, pos.z);
	}
};

function BallisticDemo(){
	this.ammoRounds = 1024;
	this.ammo = [];
	for(var i = 1; i <= this.ammoRounds; i++){
		this.ammo.push(new AmmoRound());
	}
	this.currentShotType = PISTOL;
}

BallisticDemo.prototype.fire = function(){

	// Find the first available round.
	var shot;
	for(var i = 0; i< this.ammoRounds; i++){
		if(this.ammo[i].type == UNUSED){
			shot = this.ammo[i];
			break;
		}
	}
	
	// If we didn't find a round, then exit - we can't fire.
	if(shot === undefined){
		return;
	}
	
	// Set the properties of the particle.
	switch(this.currentShotType){
		case PISTOL:
			shot.particle.setMass(2.0); // 2.0kg
			shot.particle.setVelocity(new APE.Vector3(0.0, 0.0, 35.0)); // 35m/s
			shot.particle.setAcceleration(new APE.Vector3(0.0, -1.0, 0.0));
			shot.particle.setDamping(0.99);
			break;
		case ARTILLERY:
			shot.particle.setMass(200.0); //200.0kg
			shot.particle.setVelocity(new APE.Vector3(0.0, 30.0, 40.0)); //50m/s
			shot.particle.setAcceleration(new APE.Vector3(0.0, -20.0, 0.0));
			shot.particle.setDamping(0.99);
			break;
		case FIREBALL:
			shot.particle.setMass(1.0); // 1.0kg - mostly blast damage
			shot.particle.setVelocity(new APE.Vector3(0.0, 0.0, 10.0)); // 5m/s
			shot.particle.setAcceleration(new APE.Vector3(0.0, 0.6, 0.0)); // Floats up
			shot.particle.setDamping(0.9);
			break;
		case LASER:
			//Note that this is the kind of laser seen in films,
			// not a realistic laser beam!
			shot.particle.setMass(0.1); // 0.1 kg - almost no weight
			shot.particle.setVelocity(new APE.Vector3(0.0, 0.0, 100.0)); //100m/s
			shot.particle.setAcceleration(new APE.Vector3(0.0, 0.0, 0.0)); // No gravity
			shot.particle.setDamping(0.99);
			break;
	}

	// Set the data common to all particle types.
	shot.particle.setPosition(new APE.Vector3(0.0, 1.5, 0.0));
	shot.startTime = new Date().getTime();
	shot.type = this.currentShotType;
	shot.mesh.visible = true;
};

BallisticDemo.prototype.update = function(){
	// Find the duration of the last frame in seconds.
	var duration = 1/60; // seconds
	if(duration <= 0.0)
		return;
	// Update the physics of each particle in turn
	for(var i = 0; i < this.ammoRounds; i++){
		if(this.ammo[i].type != UNUSED){
			// Run the physics
			this.ammo[i].particle.integrate(duration);
			//Check if the particle is now invalid
			if(this.ammo[i].particle.getPosition().y < 0.0 ||
				this.ammo[i].startTime + 5000 < new Date().getTime()||
				this.ammo[i].particle.getPosition().x > 200.0)
			{
				// We simply set the shot type to be unused, so the
				// memory it occupies can be reused by another shot.
				this.ammo[i].type = UNUSED;
				this.ammo[i].mesh.visible = false;
			}
		}
	}
};

BallisticDemo.prototype.mouse = function(){
	//fire the current weapon;
	this.fire();
};

BallisticDemo.prototype.key = function(key){
	switch(key){
		case '1' : 
			this.currentShotType = PISTOL; 
			break;
		case '2' :
			this.currentShotType = ARTILLERY;
			break;
		case '3' :
			this.currentShotType = FIREBALL;
			break;
		case '4':
			this.currentShotType = LASER;
			break;
	}
};

BallisticDemo.prototype.display = function(){
	for(var i = 0; i < this.ammoRounds; i++){
		this.ammo[i].render();
	}
	var ref = document.getElementById('OSD');
	ref.innerHTML = "Click: Fire <br> 1-4: Select Ammo <br>";
	
	switch(this.currentShotType){
		case PISTOL:
			ref.innerHTML += "Current Ammo: Pistol";
			break;
		case ARTILLERY:
			ref.innerHTML += "Current Ammo: Artillery";
			break;
		case FIREBALL:
			ref.innerHTML += "Current Ammo: Fireball";
			break;
		case LASER:
			ref.innerHTML += "Current Ammo: Laser";
			break;
	}
	
};