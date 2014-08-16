/**
  * A contact represents two objects in contact (in this case 
  * ParticleContact representing two particles). Resolving a
  * contact removes their interpretation, and applies sufficient
  * impulse to keep them apart. Colliding bodies may also rebound.
  *
  * The contact has no callable functions, it just holds the contact details.
  * To resolve a set of contacts, use the contact resolver class.
  */


APE.ParticleContact = function(){
	/**
	  * Holds the particles that are involved in the contact. The
	  * second of these can be NULL for contacts with the scenery.
	  */
	this.particle = [];
	
	/** 
	  * Holds the normal restitution coefficient at the contact.
	  */
	this.restitution = 1;
	
	/**
	  * Holds the direction of the contact in world coordinates.
	  */
	this.contactNormal = new APE.Vector3();
	
	/**
	  * Holds the depth of the penetration at the contact.
	  */
	this.penetration = 0;
	
	/**
	  * Holds the amount each particle is moved during interpenetration.
	  */
	this.particleMovement = [new APE.Vector3(), new APE.Vector3()];
	
};

APE.ParticleContact.prototype = {
	constructor: APE.ParticleContact,

	/** 
	  * Resolves this contact for both velocity and interpenetration.
      */
	resolve : function(duration){
		this.resolveVelocity(duration);
		this.resolveInterpenetration(duration);
	},
	
	/**
	  * Calculates the separating velocity at this contact.
	  */
	calculateSeparatingVelocity : function(){
		var relativeVelocity = this.particle[0].getVelocity();
		if(this.particle[1]){
			relativeVelocity = relativeVelocity.sub(this.particle[1].getVelocity());
		}
		return relativeVelocity.dot(this.contactNormal);
	},
	
	/**
	  * Handles the impulse calculations for this collision.
	  */
	resolveVelocity : function(duration){
		// Find the velocity in the direction of the contact.
		var separatingVelocity = this.calculateSeparatingVelocity();

		// Check if it needs to be resolved.
		if(separatingVelocity > 0){
			// The contact is either separating, or stationary;
			// no impulse is required.
			return;
		}

		// Calculate the new separating velocity.
		var newSepVelocity = - separatingVelocity * this.restitution;
		
		// Check the velocity build-up due to acceleration only.
		var accCausedVelocity = this.particle[0].getAcceleration();
		if(this.particle[1]){
			accCausedVelocity = accCausedVelocity.sub(this.particle[1].getAcceleration());
		}
		var accCausedSepVelocity = accCausedVelocity.dot(this.contactNormal) * duration;
		
		//If we've got a closing velocity due to acceleration build-up,
		// remove it from the new separation velocity.
		if(accCausedSepVelocity < 0){
			newSepVelocity += this.restitution * accCausedSepVelocity;
			// Make sure we haven't removed more than there was
			// to remove;
			if(newSepVelocity < 0){
				newSepVelocity = 0;
			}
		}
		
		var deltaVelocity = newSepVelocity - separatingVelocity;
		
		// We apply the change in velocity to each object in proportion to
		// their inverse mass ( those with lower inverse mass [higher actual mass]
		// get less change in velocity).
		var totalInverseMass = this.particle[0].getInverseMass();
		if(this.particle[1]){
			totalInverseMass += this.particle[1].getInverseMass();
		}
		
		// If all particles have infinite mass, the impulses have no effect.
		if(totalInverseMass <= 0){
			return ;
		}
		
		// Calculate the impulse to apply.
		var impulse  = deltaVelocity / totalInverseMass;
		
		// Find the amount of impulse per unit of inverse mass.
		var impulsePerIMass = this.contactNormal.multiplyScalar(impulse);
		
		
		
		// Apply impulses: they are applied in the direction of the contact,
		// and are proportional to the inverse mass.
		this.particle[0].setVelocity(this.particle[0].getVelocity().add(impulsePerIMass.multiplyScalar(this.particle[0].getInverseMass())));
		if(this.particle[1]){
			// particle 1 goes in the opposite direction
			this.particle[1].setVelocity(this.particle[1].getVelocity().add(impulsePerIMass.multiplyScalar(-this.particle[1].getInverseMass())));
		}
		
		
	},
	
	/**
	  * Handles the interpenetration resolution for this contact.
	  */
	resolveInterpenetration : function(duration){
		// If we don't have any penetration, skip this step.
		if(this.penetration <= 0){
			return;
		}
		
		// The movement of each of object is based on their mass,
		// so total that.
		var totalInverseMass = this.particle[0].getInverseMass();
		if(this.particle[1]){
			totalInverseMass += this.particle[1].getInverseMass();
		}
		
		// If all particles have infinite mass, then we do nothing.
		if(totalInverseMass <= 0 ){
			return;
		}
		
		// Find the amount of penetration resolution per unit
		// of inverse mass.
		
		var movePerIMass = this.contactNormal.multiplyScalar(this.penetration/ totalInverseMass);
		
		// Calculate the movement amounts.
		this.particleMovement[0] = movePerIMass.multiplyScalar(this.particle[0].getInverseMass());
		if(this.particle[1]){
			this.particleMovement[1] = movePerIMass.multiplyScalar(-this.particle[1].getInverseMass());
		}else{
			this.particleMovement[1] = new APE.Vector3(0,0,0);
		}
		
		//Apply the penetration resolution.
		this.particle[0].setPosition(this.particle[0].getPosition().add(this.particleMovement[0]));
		if(this.particle[1]){
			this.particle[1].setPosition(this.particle[1].getPosition().add(this.particleMovement[1]));
		}
	}
};


/** 
  * The contact resolution routine for particle contacts. One
  * resolver instance can be shared for the entire simulation.
  */

/** Creates a new contact resolver. */

APE.ParticleContactResolver = function(iterations){
	/** 
	  * Holds the number of iterations allowed.
	  */
	this.iterations = iterations || 0;
	
	/** 
	  * This is a performance tracking value; we keep a record
	  * of the actual number of iterations used.
	  */
	this.iterationsUsed = 0;
};

APE.ParticleContactResolver.prototype = {
	constructor :  APE.ParticleContactResolver,
	
	/** 
	  * Sets the number of iterations that can be used.
	  */
	setIterations : function(iterations){
		this.iterations = iterations;
	},
	
	/** 
	  * Resolves a set of particle contacts for both penetration
	  * and velocity.
	  */
	resolveContacts :  function(contactArray, numContacts, duration){
		var i;
		this.iterationsUsed = 0;
		while(this.iterationsUsed < this.iterations){
			// Find the contact with the largest closing velocity.
			var max = Number.MAX_VALUE;
			var maxIndex = numContacts;
			for(i = 0; i< numContacts; i++){
				var sepVel = contactArray[i].calculateSeparatingVelocity();
				if(sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0)){
					max = sepVel;
					maxIndex = i;
				}
			}
			
			// Do we have anything worth resolving?
			if(maxIndex === numContacts){
				break;
			}

			//Resolve this contact.
			contactArray[maxIndex].resolve(duration);
			
			//Update the interpenetration's for all particles
			
			var move = contactArray[maxIndex].particleMovement;

			for(i = 0; i < numContacts; i++){
				if (contactArray[i].particle[0] === contactArray[maxIndex].particle[0]){
					contactArray[i].penetration -= move[0].dot(contactArray[i].contactNormal);
				}else if (contactArray[i].particle[0] === contactArray[maxIndex].particle[1]){
					contactArray[i].penetration -= move[1].dot(contactArray[i].contactNormal);
				}
				
				if (contactArray[i].particle[1]){
					if (contactArray[i].particle[1] === contactArray[maxIndex].particle[0]){
						contactArray[i].penetration += move[0].dot(contactArray[i].contactNormal);
					}
					else if (contactArray[i].particle[1] === contactArray[maxIndex].particle[1]){
						contactArray[i].penetration += move[1].dot(contactArray[i].contactNormal);
					}
				}
			}
			this.iterationsUsed++;
		}
	}
};
