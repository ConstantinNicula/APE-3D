/**
 * A force generator that applies a gravitational force. One instance
 * can be used for multiple particles.
 */
 
/** Creates the generator with the given acceleration. */ 

APE.ParticleGravity = function(gravity){
	/** Holds the acceleration due to gravity.*/
	this.gravity = gravity;
};

//Inherit Particle Force Generator
APE.ParticleGravity.prototype = {
	constructor: APE.ParticleGravity,
	
	/** Applies the gravitational force to the given Particle.*/
	updateForce : function( particle, duration ){
		// Check that we don not have infinite mass.
		if(!particle.hasFiniteMass()){
			return;
		}
		// Apply the mass-scaled force to the particle.
		particle.addForce(this.gravity.multiplyScalar(particle.getMass()));
	}
};


/**
 * A force generator that applies a drag force. One instance
 * can be used for multiple particles.
 */

/** Creates the generator with the given coefficients. */

APE.ParticleDrag = function(k1, k2){
	/** Holds the velocity drag coefficient.*/
	this.k1 = k1;
	/** Holds the velocity squared drag coefficient. */
	this.k2 = k2;
}; 

APE.ParticleDrag.prototype = {
	constructor: APE.ParticleDrag,
	
	/** Applies the drag force to the given particle. */
	updateForce: function( particle, duration ){
		var force = particle.getVelocity();

		//Calculate the total drag coefficient.
		var dragCoeff = force.magnitude();
		dragCoeff = this.k1 * dragCoeff + this.k2 * dragCoeff * dragCoeff;
		
		//Calculate the final force and apply it.
		force.normalize();
		force = force.multiplyScalar(-dragCoeff);
		particle.addForce(force);
	}
};


/**
  * A force generator that applies a spring force.
  */
 
 /** Creates a new spring wit the given parameters*/
 
APE.ParticleSpring = function(other, springConstant, restLength){
	/** The particle at the other end of the spring. */
	this.other = other;
	
	/** Holds the spring constant. */
	this.springConstant = springConstant;
	
	/** Holds the rest length of the spring. */
	this.restLength = restLength;
};

APE.ParticleSpring.prototype = {
	constructor: APE.ParticleSpring,
	
	/** Applies the spring force to the given particle. */
	updateForce: function(particle, duration){
		// Calculate the vector of the spring.
		var force = particle.getPosition();
		force = force.sub(this.other.getPosition());
		
		// Calculate the magnitude of the force.
		var magnitude = force.magnitude();
		magnitude = (this.restLength - magnitude) * this.springConstant;
		
		// Calculate the final force and apply it.
		force.normalize();
		force = force.multiplyScalar(magnitude);
		particle.addForce(force);
	}
};

/** 
  * A force generator that applies a spring force, where
  * one end is attached to a fixed point in space.
  */

/** Creates a new spring with the given parameters.*/
APE.ParticleAnchoredSpring = function(anchor, springConstant, restLength){
	/** The location of the anchored end of the spring. */	
	this.anchor = anchor;
	
	/** Holds the spring constant. */
	this.springConstant = springConstant;
	
	/** Holds the rest length of the spring. */
	this.restLength = restLength;
};


APE.ParticleAnchoredSpring.prototype = {
	constructor: APE.ParticleAnchoredSpring,
	/** Applies the spring force to the given particle. */
	updateForce : function(particle, duartion){
		// Calculate the vector  of the spring.
		var force = particle.getPosition();
		force = force.sub(this.anchor);
		
		// Calculate the magnitude of the force.
		var magnitude = force.magnitude();
		magnitude = (this.restLength - magnitude) * this.springConstant;
		
		// Calculate the final force and apply it.
		force.normalize();
		force = force.multiplyScalar(magnitude);
		particle.addForce(force);
	}
};

/** 
  * A force generator that applies a spring force only
  * when extended.
  */

 /** Creates a new bungee with the given parameters. */
 APE.ParticleBungee = function(other, springConstant, restLength){
	/** The particle at the other end of the spring. */
	this.other = other;
	/** Holds the spring constant. */
	this.springConstant = springConstant;
	/**
	  * Holds the length of the bungee at the point it begins to 
	  * generate a force.
	  */
	this.restLength = restLength;
 };

 APE.ParticleBungee.prototype = {
	constructor: APE.ParticleBungee,
	
	 /** Applies the spring force to the given particle. */
	updateForce : function(particle, duration){
		//Calculate the vector of the spring.
		var force = particle.getPosition();
		force = force.sub(this.other.getPosition());
		
		// Check if the bungee is compressed.
		var magnitude = force.magnitude();
		if(magnitude <= this.restLength){
			return;
		}
		
		// Calculate the magnitude of the force.
		magnitude = this.springConstant * (this.restLength - magnitude);
		
		// Calculate the final force and apply it.
		force.normalize();
		force = force.multiplyScalar(magnitude);
		particle.addForce(force);
	}
 };
 
 /**
   * A force generator that applies a buoyancy for a plane of 
   * liquid parallel to XZ plane.
   */

/** Creates a new buoyancy force with the given parameters. */
APE.ParticleBuoyancy = function(maxDepth, volume, waterHeight, liquidDensity){
	/**
	  * The maximum depth of the object before it generates
	  * its maximum buoyancy force.
	  */
	this.maxDepth = maxDepth;
	
	/** 
	  * The volume of the object.
	  */
	this.volume = volume;
	
	/**
	  * The height of the water plane above y = 0. The plane will be
	  * parallel to XZ plane.
	  */
	this.waterHeight =  waterHeight;
	
	/** 
	  * The density of the liquid. Pure water has a density
	  * of 1000 kg per cubic meter.
	  */
	this.liquidDensity = liquidDensity || 1000;
};
 
APE.ParticleBuoyancy.prototype = {
	constructor: APE.ParticleBuoyancy, 
	
	/** Applies the buoyancy force to the given particle. */
	updateForce : function(particle, duration){
		// Calculate the submersion depth;
		var depth = particle.getPosition().y;
		
		//Check if we're out of the water.
		if(depth >= this.waterHeight + this.maxDepth){
			return;
		}
		
		var force = new APE.Vector3();
		// Check if we're at maximum depth.
		if(depth <= this.waterHeight - this.maxDepth){
			force.y = this.liquidDensity * this.volume;
			particle.addForce(force);
			return ;
		}
		
		// Otherwise we are partly submerged.
		force.y = this.liquidDensity * this.volume * (this.waterHeight - depth + this.maxDepth)/ ( 2 * this.maxDepth);
		particle.addForce(force);
	}
};

/**
  * A force generator that fakes a stiff spring force, and where
  * one end is attached to a fixed point in space.
  */

/** Creates a new spring with the given parameters. */  
APE.ParticleFakeSpring = function(anchor, springConstant, damping){
	/** The location of the anchored end of the spring. */
	this.anchor = new APE.Vector3();
	
	/** Holds the spring constant. */
	this.springConstant = springConstant;
	
	/** Holds the damping on the oscillation of the spring. */
	this.damping = damping;
};

APE.ParticleFakeSpring.prototype = {
	constructor: APE.ParticleFakeSpring,
	
	/** Applies the spring force to the given particle. */
	updateForce: function(particle, duration){
		//Check that we do not have infinite mass.
		if(!particle.hasFiniteMass()){
			return;
		}
		// Calculate the relative position of the particle to the anchor.
		var position = particle.getPosition();
		position = position.sub(this.anchor);
		
		// Calculate the constants and check that they are in bounds.
		var gamma = 0.5 * Math.sqrt(4 * this.springConstant - this.damping * this.damping);
		if(gamma === 0.0){
			return ;
		}
		var c = new APE.Vector3();
		c = position.multiplyScalar(this.damping/(2 * gamma));
		c = c.add(particle.getVelocity().multiplyScalar(1/ gamma));
		
		//Calculate the target position.
		var target =  position.multiplyScalar(Math.cos(gamma * duration));
		target = target.add(c.multiplyScalar(Math.sin(gamma * duration)));
		target = target.multiplyScalar(Math.exp(-0.5 * duration * this.damping));
		
		
		// Calculates the resulting acceleration, and therefore the force.
		var accel = target.sub(position).multiplyScalar(1/(duration * duration));
		accel = accel.sub(particle.getVelocity().multiplyScalar(1/duration));
		particle.addForce(accel.multiplyScalar(particle.getMass()));
	}
};


/**
  * Keeps track of one force generator and the particle it
  * applies to.
  */
  
APE.ParticleForceRegistration = function(particle, fg){
	this.particle = particle;
	this.fg = fg;
};

/**
  * Holds all the force generators and the particle that they apply to.
  */
  
APE.ParticleForceRegistry = function(){
	/**
	  * Holds the list of registrations.
	  */
	this.registrations = [];
 };
 
 APE.ParticleForceRegistry.prototype = {
 
	constructor: APE.ParticleForceRegistry,
	
	/**
	  * Registers the given force generator to apply to the
	  * given particle.
	  */
	add : function(particle, fg){
		var registration = new APE.ParticleForceRegistration( particle, fg);
		this.registrations.push(registration);
	},
	
	/** 
	  * Removes the given registered pair from the registry.
	  * If the pair is not registered, this method will have
	  * no effect.
	  */
	remove : function(particle, fg){
		var registration, found = false;
		for(var i = 0; i < this.registrations.length && !found; i++){
			registration =  this.registrations[i];
			if(registration.particle === particle && registration.fg === fg){
				this.registrations.splice(i,1);
				found = true;
			}
		}
	},
	
	/**
	  * Clears all registrations from the registry. This will
	  * not delete the particles or the force generators
      * themselves, just the records of their connection.	  
	  */
	clear : function(){
		this.registrations = [];
	},
	
	/**
	  * Calls all the force generators to update the forces of their
	  * corresponding particles.
	  */
	updateForces : function(duration){
		var registration;
		for(var i = 0; i < this.registrations.length; i++){
			registration = this.registrations[i];
			registration.fg.updateForce(registration.particle, duration);
		}
	}
 };