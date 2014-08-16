/**
  * Keeps track of a set of particle, and provides means to
  * update them all.
  */
  
/**
  * Creates a new particle simulator that can handle up to the
  * given number of contacts per frame. You can also optionally
  * give a number of contact-resolution iterations to use. If you
  * don't give a number of iterations, then twice the number of
  * contacts will be used.
  */
APE.ParticleWorld = function(maxContacts, iterations){
	/**
	  * Holds the particles.
	  */
	this.particles = [];
	
	/**
	  * True if the world should calculate the number of iterations
	  * to give the contact resolver at each frame.
	  */ 
	this.calculateIterations = iterations === undefined;
	
	/** 
      * Holds the force generators for the particles in the world.
	  */
	this.registry = new APE.ParticleForceRegistry();
	
	/**
	  * Holds the resolver for contacts.
	  */
	this.resolver = new APE.ParticleContactResolver(iterations);
	
	/**
	  * Contact generators.
	  */
	this.contactGenerators = [];
	
	/**
	  * Holds the list of contacts.
	  */
	this.contacts = [];
	
	/**
	  * Holds the maximum number of contacts allowed (the 
	  * size of the contacts array).
	  */
	this.maxContacts = maxContacts;
};

APE.ParticleWorld.prototype = {
	constructor: APE.ParticleWorld,
	/**
	  * Initializes the world for a simulation frame. This clears
	  * the force accumulators for particles in the world. After 
	  * calling this, the particles can have their forces for this 
	  * frame added.
	  */
	startFrame : function(){
		for(var i = 0; i < this.particles.length; i++){
			//Remove all forces from the accumulator.
			this.particles[i].clearAccumulator();
		}
		
		for(var i = 0; i< this.maxContacts; i++){
			this.contacts.push(new APE.ParticleContact());
		}
	},
	
	/**
      * Calls each of the registered contact generators to report
	  * their contacts. Returns the number of generated contacts.
	  */
	generateContacts : function(){
		var limit = this.maxContacts;
		var nextContact = 0;
		for(var i = 0; i < this.contactGenerators.length; i++){
			var used;
			if(this.contactGenerators[i] instanceof APE.GroundContacts){
				used = this.contactGenerators[i].addContact(this.contacts, nextContact, limit);
			}else{
				used = this.contactGenerators[i].addContact(this.contacts[nextContact], limit);
			}
			limit -= used;
			nextContact += used;
			
			// We've run out of contacts to fill. This means we're missing
			// contacts.
			if(limit <= 0){
				break;
			}
		}
		
		// Return the number of contacts used;
		return this.maxContacts - limit;
	},
	
	/**
	  * Integrates all the particles in this world forward in time
	  * by the given duration.
	  */
	integrate : function(duration){
		for(var i=0; i < this.particles.length; i++){
			// Integrate the particle by the given duration.
			this.particles[i].integrate(duration);
		}
	},
	
	/**
	  * Processes all the physics for the particle world.
	  */
	runPhysics : function(duration){
		// First, apply the force generators.
		this.registry.updateForces(duration);
		
		// Then integrate the objects.
		this.integrate(duration);
		
		// Generate contacts.
		var usedContacts = this.generateContacts();
		
		// And process them.
			
		if(usedContacts){
			if(this.calculateIterations){
				this.resolver.setIterations(usedContacts * 2);
			}
			this.resolver.resolveContacts(this.contacts, usedContacts, duration);
		}
	},
	
	/**
	  * Returns the list of particles
	  */
	getParticles : function(){
		return this.particles;
	},
	
	/**
	  * Get the list of contact generators.
	  */
	getContactGenerators : function(){
		return this.contactGenerators;
	},
	
	/**
	  * Returns the force registry.
	  */
	getForceRegistry : function(){
		return this.registry;
	} 
};

/**
  * A contact generator that takes a vector of particles and
  * collides them against the ground.
  */
  
APE.GroundContacts = function(){
	this.particles = [];
};

APE.GroundContacts.prototype = {
	constructor : APE.GroundContacts,
	
	init: function(particles){
		this.particles = particles;
	},
	
	addContact : function(contacts, nextContact, limit){
		var count = 0;
		for(var i = 0; i < this.particles.length; i++){
			var y = this.particles[i].getPosition().y;
			if(y < 0){
				contacts[nextContact].contactNormal = new APE.Vector3(0, 1, 0);
				contacts[nextContact].particle[0] =  this.particles[i];
				contacts[nextContact].particle[1] = null;
				contacts[nextContact].penetration = -y;
				contacts[nextContact].restitution = 0.2;
				nextContact++;
				count++;
			}
			if(count >= limit){
				return count;
			}
		}
		return count;
	}
};