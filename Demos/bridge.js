var ROD_COUNT = 6,
    CABLE_COUNT = 10,
	SUPPORT_COUNT = 12,
	BASE_MASS = 1,
	EXTRA_MASS = 10;
function BridgeDemo(){

	this.supports = [];
	this.cables = [];
	this.rods = [];
	
	this.massPos = new APE.Vector3(0,0,0.5);
	this.massDisplayPos = new APE.Vector3();
	
	this.world = new APE.ParticleWorld(100);
	this.particleArray = [];
	this.groundContactGenerator = new APE.GroundContacts;
	
	
	// Create the masses and connections.
	for(var i = 0; i < 12; i++){
		this.particleArray[i] = new APE.Particle();
		this.particleArray[i].setPosition(new APE.Vector3(
			Math.floor(i/2) * 2.0 -5.0,
			4,
			(i%2) * 2.0 -1.0
		));
		this.particleArray[i].setVelocity(new APE.Vector3());
		this.particleArray[i].setDamping(0.9);
		this.particleArray[i].setAcceleration(new APE.Vector3(0, -9.81, 0));
		this.particleArray[i].clearAccumulator();
	}
	
	// Add the links
	for(var i = 0; i < 10; i++){
		this.cables[i] = new APE.ParticleCable();
		this.cables[i].particle[0] = this.particleArray[i];
		this.cables[i].particle[1] = this.particleArray[i+2];
		this.cables[i].maxLength = 1.9;
		this.cables[i].restitution = 0.3;
		this.world.getContactGenerators().push(this.cables[i]);
 	}
	
	for(var i = 0; i < SUPPORT_COUNT; i++){
		this.supports[i] = new APE.ParticleCableConstraint();
		this.supports[i].particle = this.particleArray[i];
		this.supports[i].anchor = new APE.Vector3(
			Math.floor(i/2) * 2.2 - 5.5,
			6,
			(i % 2) * 1.6 - 0.8
		);
		if(i<6){
			this.supports[i].maxLength = Math.floor(i/2)*0.5 + 3;
		}else{
			this.supports[i].maxLength = 5.5 -  Math.floor(i/2) * 0.5;
		}
		this.supports[i].restitution = 0.5;
		this.world.getContactGenerators().push(this.supports[i]);		
	}
	
	for(var i = 0 ; i < ROD_COUNT; i++){
		this.rods[i] = new APE.ParticleRod();
		this.rods[i].particle[0] = this.particleArray[i * 2];
		this.rods[i].particle[1] = this.particleArray[i * 2 + 1];
		this.rods[i].length = 2;
		this.world.getContactGenerators().push(this.rods[i]);
	}
	
	this.updateAdditionalMass();
}

/**
  * Updates particle masses to take into account the mass
  * that's crossing the bridge.
  */

BridgeDemo.prototype.updateAdditionalMass = function(){
	for(var i = 0; i < 12; i++){
		this.particleArray[i].setMass(BASE_MASS);
	}
	// Find the coordinates of the mass as an index and proportion
	var x = 0 | this.massPos.x;
	var xp = this.massPos.x % 1;
	if( x < 0){
		x = 0;
		xp = 0; 		
	}
	if( x >= 5){
		x = 5;
		xp = 0;
	}
	var z = 0 | this.massPos.z;
	var zp = this.massPos.z % 1;
	
	if( z < 0){
		z = 0;
		zp = 0;
	}
	if( z >= 1){
		z = 1;
		zp = 0;
	}
	// Calculate where to draw the mass
	this.massDisplayPos.clear();
	// Add the proportion to the correct masses;
	this.particleArray[x*2+z].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*(1-zp));
	this.massDisplayPos.addScaledVector(
		this.particleArray[x*2+z].getPosition(),
		(1-xp )*(1-zp)
	);
	if(xp>0){
		this.particleArray[x*2+z+2].setMass(BASE_MASS + EXTRA_MASS*xp*(1-zp));
		this.massDisplayPos.addScaledVector(
			this.particleArray[x*2+z+2].getPosition(),
			xp*(1-zp)
		);
		
		if(zp>0){
			this.particleArray[x*2+z+3].setMass(BASE_MASS + EXTRA_MASS*xp*zp);
			this.massDisplayPos.addScaledVector(
				this.particleArray[x*2+z+3].getPosition(),
				xp*zp
			);
		}
	}
	if(zp>0){
		this.particleArray[x*2+z+1].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*zp);
		this.massDisplayPos.addScaledVector(
			this.particleArray[x*2+z+1].getPosition(),
			(1-xp)*zp
		);
	}
};

BridgeDemo.prototype.key = function(key){
	switch(key){
		case 's': case'S':
			this.massPos.z += 0.1;
			if(this.massPos.z > 1)
				this.massPos.z = 1;
			break;
		case 'w': case'W':
			this.massPos.z -= 0.1;
			if(this.massPos.z < 0)
				this.massPos.z = 0;
			break;
		case 'a': case'A':
			this.massPos.x -= 0.1;
			if(this.massPos.x < 0)
				this.massPos.x = 0;
			break;
		case 'd': case'D':
			this.massPos.x += 0.1;
			if(this.massPos.x > 5)
				this.massPos.x = 5;
			break;
	}
};