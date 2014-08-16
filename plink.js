/**
  * Links connect two particles together, generating a contact if
  * they violate the constraints of their link. It is used as
  * base class for cables and rods, and could be used as a base
  * class for springs with a limit to their extension.
  */
  
APE.ParticleLink = function(){
	/** 
	  * Holds the pair of particles that are connected by this link.
	  */
	this.particle = [];
};

APE.ParticleLink.prototype = {
	constructor: APE.ParticleLink,
	
	/**
	  * Returns the current length of the link.
	  */
	  
	currentLength : function(){
		var relativePos = this.particle[0].getPosition().sub(this.particle[1].getPosition());
		return relativePos.magnitude();
	},
	
	/**
	  * Generates the contacts to keep this link from being 
	  * violated. This class can only ever generate a single 
	  * contact, so the pointer can be a pointer to a single 
	  * element, the limit parameter is assumed to be at least 1
	  * and return the value 0 if the cable wasn't over extended, or
	  * 1 if a contact was needed.
	  *
	  */
	addContact : function( contact, limit){
		// This method will be overloaded in the subclass.
		return 1;
	}
};

/** 
  * Cables link a pair of particles, generating a contact if they
  * stray too far apart.
  */ 
APE.ParticleCable = function(){
	APE.ParticleLink.call(this);
	/**
	  * Holds the maximum length of the cable.
	  */
	this.maxLength = 0;
	
	/**
	  * Holds the restitution (bounciness) of the cable.
	  */
	this.restitution = 1;
};

APE.extend(APE.ParticleCable, APE.ParticleLink);
/**
  * Fills the given contact structure with the contact needed
  * to keep the cable from overExtending.
  */
APE.ParticleCable.prototype.addContact = function(contact, limit){
	// Find the length of the cable.
	var length = this.currentLength();

	//console.log(length);
	// Check if we're overextended.
	if(length < this.maxLength){
		return 0;
	}
	
	//Otherwise, return the contact.
	contact.particle[0] = this.particle[0];
	contact.particle[1] = this.particle[1];
	
	// Calculate the normal.
	var normal = this.particle[1].getPosition().sub(this.particle[0].getPosition());
	normal.normalize();
	contact.contactNormal = normal;
	
	contact.penetration = length - this.maxLength;
	contact.restitution = this.restitution;
	
	
	return 1;
};

/**
  * Rods link a pair of particles, generating a contact if they 
  * stray too far apart or too close.
  */
APE.ParticleRod = function(){
	APE.ParticleLink.call(this);
	/**
	  * Holds the length of the rod.
	  */
	this.length = 0;
};

APE.extend(APE.ParticleRod, APE.ParticleLink);

/**
  * Fills the given contact structure with the contact needed
  * to keep the rod from extending or compressing.
  */
APE.ParticleRod.prototype.addContact = function(contact, limit){
	// Find the length of the rod.
	var currentLen = this.currentLength();
	
	// Check if we're overextended.
	if(currentLen === this.length){
		return 0;
	}
	
	// Otherwise, return the contact.
	contact.particle[0] = this.particle[0];
	contact.particle[1] = this.particle[1];
	
	// Calculate the normal.
	var normal = this.particle[1].getPosition().sub(this.particle[0].getPosition());
	normal.normalize();
	
	// The contact normal depend on whether we're extending or compressing.
	if(currentLen > this.length){
		contact.contactNormal = normal;
		contact.penetration = currentLen - this.length;
	}else{
		contact.contactNormal = normal.multiplyScalar(-1);
		contact.penetration = this.length - currentLen;
	}
	
	// Always use zero restitution (no bounciness).
	contact.restitution = 0;
	
	return 1;
};

/**
  * Constraints are just like links, except the connect a particle to 
  * an immovable anchor point.
  */

APE.ParticleConstraint = function() {
	/**
	  * Holds the particles connected by this constraint.
	  */
	this.particle = null;

	/**
	  * The point to which the particle is anchored.
	  */
	this.anchor = new APE.Vector3();
};

APE.ParticleConstraint.prototype = {
	constructor: APE.ParticleConstraint,
	
	/**
	  * Return the current length of the link.
	  */
	currentLength : function(){
		var relativePos = this.particle.getPosition().sub(this.anchor);
		return relativePos.magnitude();
	},
	/**
	  * Generates the contacts to keep this link from being 
	  * violated. This class can only ever generate a single 
	  * contact, so the pointer can be a pointer to a single 
	  * element, the limit parameter is assumed to be at least 1
	  * and return the value 0 if the cable wasn't over extended, or
	  * 1 if a contact was needed.
	  *
	  */
	addContact : function (contact, limit){
		// To be overloaded in the subclass.
	}
};

/** 
  * Cables link a particle to an anchor point, generating a contact if they
  * stray too far apart.
  */

APE.ParticleCableConstraint = function(){
	APE.ParticleConstraint.call(this);
	
	/**
	  * Holds the maximum length of the cable.
	  */
	this.maxLength = 0;
	
	/**
	  * Holds the restitution (bounciness) of the cable.
	  */
	this.restitution = 1;
};

APE.extend(APE.ParticleCableConstraint, APE.ParticleConstraint);

/**
  * Fills the given contact structure with the contact needed
  * to keep the cable from over-extending.
  */


APE.ParticleCableConstraint.prototype.addContact = function(contact, limit){
	// Find the length of the cable.
	var length = this.currentLength();
	
	// Check if we're over-extended
	if(length < this.maxLength){
		return 0;
	}
	
	// Otherwise return the contact
	contact.particle[0] = this.particle;
	contact.particle[1] = null;
	
	// Calculate the normal
	var normal = this.anchor.sub(this.particle.getPosition());
	normal.normalize();
	contact.contactNormal = normal;
	
	contact.penetration = length - this.maxLength;
	contact.restitution = this.restitution;
	
	return 1;
};

/** 
  * Rods link particle and to an anchor point, generating a contact if they
  * stray to far apart or too close.
  */

APE.ParticleRodConstraint = function(){
	APE.ParticleConstraint.call(this);
	
	/** 
	  * Holds the length of the rod.
	  */
	this.length = 0;
};

APE.extend(APE.ParticleRodConstraint, APE.ParticleConstraint);

/**
  * Fills the given contact structure with the contact needed
  * to keep  the rod from extending or compressing.
  */
APE.ParticleRodConstraint.prototype.addContact = function(contact, limit){
	// Find the length of the rod
	var currentLen =  this.currentLength();
	
	//Check if we're over-extended
	if(currentLen === this.length){
		return 0;
	}
	
	// Otherwise return the contact.
	contact.particle[0] = this.particle;
	contact.particle[1] = null;
	
	// Calculate the normal
	var normal = this.anchor.sub(this.particle.getPosition());
	normal.normalize();
	
	// The contact normal depends on whether we're extending or compressing
	if(currentLen > this.length){
		contact.contactNormal = normal;
		contact.penetration = currentLen - this.length;
	}else{
		contact.contactNormal = normal.multiplyScalar(-1);
		contact.penetration = this.length - currentLen;
	}
	
	// Always use zero restitution (no bounciness)
	contact.restitution = 0;
	
	return 1;
};

