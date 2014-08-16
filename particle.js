/**
  * A particle is the simplest object that can be simulated in the
  * physics system.
  */
  
APE.Particle = function(){
	/**
	  * Holds the linear position of the particle in
	  * world space.
	  */
	this.position = new APE.Vector3();
	
	/**
	  * Holds the linear velocity of the particle in
	  * world space.
	  */
	this.velocity = new APE.Vector3();
	
	/**
	  * Holds the acceleration of the particle. This value
	  * can be used to set acceleration due to gravity
	  * (its primary use), or any other constant acceleration.
	  */
	this.acceleration = new APE.Vector3();
	
	/**
	  * Holds the amount of damping applied to linear
	  * motion. Damping is required to remove energy added
	  * through numerical instability in the integrator
	  */
	this.damping =  1;
	
	/**
	  * Holds the inverse of the mass of the particle. It
	  * is more useful to hold the inverse mass because
	  * integration is simpler, and because in real-time
	  * simulation it is more useful to have objects with
	  * infinite mass(immovable) than zero mass
	  *(completely unstable in numerical simulation).
	  */
	this.inverseMass = 1;
	
	/** 
	  * Holds the accumulated force to be applied at the next
	  * simulation iteration only. THis value is zeroed at each
	  * integration step.
	  */
	this.forceAccum = new APE.Vector3();
};


APE.Particle.prototype = {
	
	constructor: APE.Particle,
	
	/**
	  * Integrates the particle forward in time by the given amount.
	  * This function uses the Newton-Euler integration method, which is a
	  * linear approximation to the correct integral. For this reason it
	  * may be inaccurate in some cases.
	  */
	  

	integrate : function (duration){
		// We don't integrate things with infinite mass.
		if(this.inverseMass <= 0){
			return;
		}
		
		//Update linear position.
		this.position.addScaledVector(this.velocity, duration);
		
		// Work out the acceleration from the force.
		var resultingAcc = this.acceleration.clone();
		resultingAcc.addScaledVector(this.forceAccum, this.inverseMass);
		
		// Update linear velocity from the acceleration
		this.velocity.addScaledVector(resultingAcc, duration);
		
		// Impose drag.
		this.velocity =  this.velocity.multiplyScalar(Math.pow(this.damping, duration));
		
		// Clear the forces.
		this.clearAccumulator();
	},
	
	/**
	  * Clears the forces applied to the particle. This will be
	  * called automatically after each integration step.
	  */
	
	clearAccumulator : function(){
		this.forceAccum.clear();
	},
	
	/**
      * Adds the given force to the particle to be applied at the 
	  * next iteration only
	  */
	addForce : function(force){
		this.forceAccum = this.forceAccum.add(force);
	},
	
	/** 
	  * Sets the mass of the particle.
	  * The new mass may not be zero. Small masses
	  * can produce unstable rigid bodies under simulation.
	  */

	setMass : function(mass){
		if(mass !== 0){
			this.inverseMass = 1/mass;
		}
	},

	/**
	  * Gets the current mass of the particle.
	  */
	  
	getMass : function(){
		if(this.inverseMass === 0){
			return Number.MAX_VALUE;
		}else{
			return 1/this.inverseMass; 
		}
	},

	/**
	  * Sets the inverse mass of the particle.
	  * This may be zero, for a body with infinite mass (unmovable).
	  */
	  
	setInverseMass :  function(inverseMass){
		this.inverseMass = inverseMass;
	},

	/**
	  * Gets the current inverse mass of the particle.
	  */
	  
	getInverseMass : function(){
		return this.inverseMass;
	},

	/**
	  * Returns true if the mass of the particle is not infinite.
	  */

	hasFiniteMass :  function(){
		return this.inverseMass > 0.0;
	},

	/**
	  * Sets the damping of the particle.
	  */

	setDamping : function(damping){
		this.damping = damping;
	},

	/**
	  * Gets the current damping value.
	  */

	getDamping : function(){
		return this.damping;
	},

	/**
	  * Sets the position of the particle.
	  * The values are set on a component basis in order
	  * to avoid referencing issues.
	  */

	setPosition : function(position){
		this.position.x = position.x;
		this.position.y = position.y;
		this.position.z = position.z;
	},

	/**
	  * Gets the position of the particle.
	  */
	  
	getPosition : function(){
		return this.position.clone();
	},

	/**
	  * Sets the velocity of the particle.
	  * The values are set on a component basis in order to
	  * avoid referencing issues.
	  */

	setVelocity : function(velocity){
		this.velocity.x = velocity.x;
		this.velocity.y = velocity.y;
		this.velocity.z = velocity.z;
	},

	/**
	  * Gets the velocity of the particle.
	  */

	getVelocity : function(){
		return this.velocity.clone();
	},

	/**
	  * Sets the constant acceleration of the particle.
	  * The values are set on a component basis in order to avoid 
	  * referencing issues.
	  */

	setAcceleration : function(acceleration){
		this.acceleration.x = acceleration.x;
		this.acceleration.y = acceleration.y;
		this.acceleration.z = acceleration.z;
	},

	/**
	  * Gets the acceleration of the particle.
	  */
	  
	getAcceleration : function(){
		return this.acceleration.clone();
	}

};