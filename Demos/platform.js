var ROD_COUNT = 15,
	BASE_MASS = 1,
	EXTRA_MASS = 10;
	
function PlatformDemo(){
	this.rods = [];
	this.particleArray = [];
	this.world = new APE.ParticleWorld(100);
	this.groundContactGenerator = new APE.GroundContacts();
	
	this.massPos = new APE.Vector3(0,0,0.5);
	this.massDisplayPos = new APE.Vector3();
	
	// Create the masses and connections.
	this.particleArray[0] = new APE.Particle();
	this.particleArray[0].setPosition(new APE.Vector3(0,0,1));
	this.particleArray[1] = new APE.Particle();
	this.particleArray[1].setPosition(new APE.Vector3(0,0,-1));
	this.particleArray[2] = new APE.Particle();
	this.particleArray[2].setPosition(new APE.Vector3(-3,2,1));
	this.particleArray[3] = new APE.Particle();
	this.particleArray[3].setPosition(new APE.Vector3(-3,2,-1));
	this.particleArray[4] = new APE.Particle();
	this.particleArray[4].setPosition(new APE.Vector3(4,2,1));
	this.particleArray[5] = new APE.Particle();
	this.particleArray[5].setPosition(new APE.Vector3(4,2,-1));
	
	for(var i = 0; i<6; i++){
		this.particleArray[i].setMass(BASE_MASS);
		this.particleArray[i].setVelocity(new APE.Vector3());
		this.particleArray[i].setDamping(0.9);
		this.particleArray[i].setAcceleration(new APE.Vector3(0, -9.81, 0));
		this.particleArray[i].clearAccumulator();
		
		this.world.particles.push(this.particleArray[i]);
	}
	
	this.rods[0] = new APE.ParticleRod();
	this.rods[0].particle[0] = this.particleArray[0];
	this.rods[0].particle[1] = this.particleArray[1];
	this.rods[0].length = 2;
	
	this.rods[1] = new APE.ParticleRod();
	this.rods[1].particle[0] = this.particleArray[2];
	this.rods[1].particle[1] = this.particleArray[3];
	this.rods[1].length = 2;
	
	this.rods[2] = new APE.ParticleRod();
	this.rods[2].particle[0] = this.particleArray[4];
	this.rods[2].particle[1] = this.particleArray[5];
	this.rods[2].length = 2;

	
	this.rods[3] = new APE.ParticleRod();
	this.rods[3].particle[0] = this.particleArray[2];
	this.rods[3].particle[1] = this.particleArray[4];
	this.rods[3].length = 7;
	
	this.rods[4] = new APE.ParticleRod();
	this.rods[4].particle[0] = this.particleArray[3];
	this.rods[4].particle[1] = this.particleArray[5];
	this.rods[4].length = 7;
	
	
	this.rods[5] = new APE.ParticleRod();
	this.rods[5].particle[0] = this.particleArray[0];
	this.rods[5].particle[1] = this.particleArray[2];
	this.rods[5].length = 3.606;
	
	this.rods[6] = new APE.ParticleRod();
	this.rods[6].particle[0] = this.particleArray[1];
	this.rods[6].particle[1] = this.particleArray[3];
	this.rods[6].length = 3.606;
	
	
	this.rods[7] = new APE.ParticleRod();
	this.rods[7].particle[0] = this.particleArray[0];
	this.rods[7].particle[1] = this.particleArray[4];
	this.rods[7].length = 4.472;
	
	this.rods[8] = new APE.ParticleRod();
	this.rods[8].particle[0] = this.particleArray[1];
	this.rods[8].particle[1] = this.particleArray[5];
	this.rods[8].length = 4.472;
	
	
	this.rods[9] = new APE.ParticleRod();
	this.rods[9].particle[0] = this.particleArray[0];
	this.rods[9].particle[1] = this.particleArray[3];
	this.rods[9].length = 4.123;
	
	this.rods[10] = new APE.ParticleRod();
	this.rods[10].particle[0] = this.particleArray[2];
	this.rods[10].particle[1] = this.particleArray[5];
	this.rods[10].length = 7.28;
	
	this.rods[11] = new APE.ParticleRod();
	this.rods[11].particle[0] = this.particleArray[4];
	this.rods[11].particle[1] = this.particleArray[1];
	this.rods[11].length = 4.899;
	
	this.rods[12] = new APE.ParticleRod();
	this.rods[12].particle[0] = this.particleArray[1];
	this.rods[12].particle[1] = this.particleArray[2];
	this.rods[12].length = 4.123;
	
	this.rods[13] = new APE.ParticleRod();
	this.rods[13].particle[0] = this.particleArray[3];
	this.rods[13].particle[1] = this.particleArray[4];
	this.rods[13].length = 7.28;
	
	this.rods[14] = new APE.ParticleRod();
	this.rods[14].particle[0] = this.particleArray[5];
	this.rods[14].particle[1] = this.particleArray[0];
	this.rods[14].length = 4.899;
	
	for(var i = 0; i< ROD_COUNT; i++){
		this.world.getContactGenerators().push(this.rods[i]);
	}
	
	this.groundContactGenerator.init(this.particleArray);
	this.world.getContactGenerators().push(this.groundContactGenerator);
	
	this.updateAdditionalMass();
}

/**
  * Updates particle masses to take into account the mass
  * that's on the platform.
  */
  
PlatformDemo.prototype.updateAdditionalMass = function(){
	for(var  i = 0; i<6; i++){
		this.particleArray[i].setMass(BASE_MASS);
	}
	//find the coordinates of the mass as a proportion
	var xp = this.massPos.x;
	if(xp < 0){
		xp = 0;
	}else if(xp > 1){
		xp = 1;
	}
	
	var zp = this.massPos.z;
	if(zp < 0){
		zp = 0;
	}else if(zp > 1){
		zp = 1;
	}
	
	//Calculate where to draw the mass.
	this.massDisplayPos.clear();
	
	//Add the proportion to the correct masses
	this.particleArray[2].setMass(BASE_MASS + EXTRA_MASS * (1-xp)*(1-zp));
	this.massDisplayPos.addScaledVector(
		this.particleArray[2].getPosition(),
		(1-xp)*(1-zp)
	);
	
	if(xp > 0){
		this.particleArray[4].setMass(BASE_MASS + EXTRA_MASS *xp * (1-zp));
		this.massDisplayPos.addScaledVector(
			this.particleArray[4].getPosition(),
			xp * (1-zp)
		);
		
		if(zp > 0){
			this.particleArray[5].setMass(BASE_MASS + EXTRA_MASS * xp *zp);
			this.massDisplayPos.addScaledVector(
				this.particleArray[5].getPosition(),
				xp*zp
			);
		}
	}
	if(zp > 0){
		this.particleArray[3].setMass(BASE_MASS + EXTRA_MASS * (1-xp) * zp);
		this.massDisplayPos.addScaledVector(
			this.particleArray[3].getPosition(),
			(1-xp) * zp
		);
	}
}

PlatformDemo.prototype.key = function(key){
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
}