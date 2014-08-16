/**
 * A force generator can be asked to be a force to one or 
 * more bodies.
 */


/**
 * A force generator that applies  a gravitational force. One
 * instance can be used for multiple rigid bodies.
 */
APE.Gravity = function(gravity){
	/** Holds the acceleration due to gravity/ */
	this.gravity = gravity;
};

APE.Gravity.prototype = {
	constructor: APE.Gravity,
	/** Applies the gravitational force to the given rigid body.*/
	updateForce: function(body, duration){
		// Check we don not have infinite mass.
		if(!body.hasFiniteMass()){
			return;
		}
		// Apply the mass-scaled force to the body.
		body.addForce(this.gravity.multiplyScalar(body.getMass()));
	}
};

/**
 * A force generator that applies a Spring force.
 */

APE.Spring = function(connectionPoint, other, otherConnectionPoint, springConstant, restLength){
	/**
	 * The point of connection of the spring, in local 
	 * coordinates.
	 */
	this.connectionPoint = connectionPoint;

	/**
	 * The point of connection of the spring to the other object,
	 * in that object's local coordinates.
	 */
	this.otherConnectionPoint = otherConnectionPoint;

	/**
	 * The rigid body at the other end of the spring.
	 */
	this.other = other;

	/** 
	 * Holds the spring constant.
	 */
	this.springConstant = springConstant;

	/**
	 * Holds the rest length of the spring.  
	 */
	this.restLength = restLength;
};



APE.Spring.prototype = {
	constructor : APE.Spring,
	/**
	 * Applies the spring force to the given rigid body.
	 */
	updateForce: function(body, duration){
		// Calculate the two ends in world space.
		var lws = body.getPointInWorldSpace(this.connectionPoint);
		var ows = this.other.getPointInWorldSpace(this.otherConnectionPoint);

		// Calculate the vector of the spring.
		var force = lws.sub(ows);

		// Calculate the magnitude of the force
		var magnitude = force.magnitude();
		magnitude = Math.abs(magnitude - this.restLength);
		magnitude *= this.springConstant;

		// Calculate the final force and apply it.
		force.normalize();
		force = force.multiplyScalar(-magnitude);
		body.addForceAtPoint(force, lws);
	}
};

/**
 * A force generator that applies an aerodynamic force.
 */
APE.Aero = function(tensor, position, windspeed){
    /**
     * Holds the aerodynamic tensor for the surface in body
     * space.
     */
    this.tensor = tensor;

    /**
     * Holds the relative position of the aerodynamic surface in
     * body coordinates.
     */
    this.position = position;

    /**
     * Holds a pointer to a vector containing the wind speed of
     * the environment. This is easier than managing a separate
     * wind speed vector per generator and having to update it
     * manually as the wind changes.
     */
    this.windspeed = windspeed;
};

APE.Aero.prototype = {
    constructor: APE.Aero,

    /**
     * Applies the force to the given rigid body.
     */
    updateForce: function(body, duration){
        this.updateForceFromTensor(body, duration, this.tensor);
    },

    /**
     *  Uses an explicit tensor matrix to update the force on
     *  the given rigid body. This is exactly the same as for updateForce
     *  only it takes an explicit tensor.
     */
    updateForceFromTensor: function(body, duration, tensor){
        // Calculate total velocity (windspeed and body's velocity).
        var velocity = body.getVelocity();
        velocity = velocity.add(this.windspeed);

        // Calculate the velocity in body coordinates.
        var bodyVel = body.getTransform().transformInverseDirection(velocity);

        // Calculate the force in body coordinates
        var bodyForce = tensor.transform(bodyVel);
        var force = body.getTransform().transformDirection(bodyForce);
        // Apply the force
        body.addForceAtBodyPoint(force, this.position);
    }
};

/**
 * A force generator with a control aerodynamic surface. This
 * requires three inertia tensors, for the two extremes and
 * 'resting' position of the control surface. The latter tensor is
 * the one inherited from the base class, the two extremes are
 * defined in this class.
 */
APE.AeroControl = function(base, min, max, position, windspeed){
    APE.Aero.call(this, base, position, windspeed);

    /**
     * The aerodynamic tensor for the surface, when the control is at
     * its maximum value.
     */
    this.maxTensor = max;

    /**
     * The aerodynamic tensor for the surface, when the control is at
     * its minimum value.
     */
    this.minTensor = min;

    /**
     * The current position of the control for this surface. This
     * should range between -1 (in which case the min Tensor value
     * is used), through 0 (where the base-class tensor value is
     * used) to +1 (where the maxTensor value is used).
     */
    this.controlSetting = 0;
};

APE.extend(APE.AeroControl, APE.Aero);

/**
 * Sets the current position of the control for this surface. This
 * should range between -1 (in which case the min Tensor value
 * is used), through 0 (where the base-class tensor value is
 * used) to +1 (where the maxTensor value is used). Values outside that
 * range give undefined results.
 */
APE.AeroControl.prototype.setControl = function(value){
    this.controlSetting = value;
};

/**
 * Calculates the final aerodynamic tensor for the current
 * control setting.
 */
APE.AeroControl.prototype.getTensor = function(){
    if(this.controlSetting <= -1){
        return this.minTensor.clone();
    }else if (this.controlSetting >= 1){
        return this.maxTensor.clone();
    }else if(this.controlSetting < 0){
        return APE.Matrix3.linearInterpolate(this.minTensor, this.tensor, this.controlSetting + 1);
    }else if(this.controlSetting >0){
        return APE.Matrix3.linearInterpolate(this.tensor, this.maxTensor, this.controlSetting)
    }else{
        return this.tensor;
    }
};

/**
 * Applies the force to the given rigid body.
 */
APE.AeroControl.prototype.updateForce = function(body, duration){
    var tensor = this.getTensor();
    this.updateForceFromTensor(body, duration, tensor);
};


/**
 * A force generator to apply a buoyant force to a rigid body.
 */
APE.Buoyancy = function(cOfB, maxDepth, volume, waterHeight, liquidDensity){
    /**
     * The maximum submersion depth of the object before
     * it generates its maximum buoyancy force.
     */
    this.maxDepth = maxDepth;

    /**
     * The volume of the object.
     */
    this.volume = volume;

    /**
     * The height of the water plane above y=0. The plane will be
     * parallel to the XZ plane.
     */
    this.waterHeight = waterHeight;

    /**
     * The density of the liquid. Pure water has a density of
     * 1000kg per cubic meter.
     */
    this.liquidDensity = (liquidDensity !== undefined)? liquidDensity : 1000 ;

    /**
     * The centre of buoyancy of the rigid body, in body coordinates.
     */
    this.centreOfBuoyancy = cOfB;
};

APE.Buoyancy.prototype = {
    constructor: APE.Buoyancy,
    /**
     * Applies the force to the given rigid body.
     */
    updateForce: function(body, duration){
        // Calculate the submersion depth
        var pointInWorld = body.getPointInWorldSpace(this.centreOfBuoyancy);
        var depth = pointInWorld.y;

        // Check is we're out of the water
        if(depth >= this.waterHeight + this.maxDepth){
            return;
        }
        var force = new APE.Vector3();
        // Check if we're at maximum depth
        if(depth <= this.waterHeight - this.maxDepth){
            force.y = this.liquidDensity * this.volume;
            body.addForceAtPoint(force, this.centreOfBuoyancy);
            return;
        }

        // Otherwise we ar partly submerged
        force.y = this.liquidDensity * this.volume *
            (this.waterHeight - depth + this.maxDepth)/ ( 2 * this.maxDepth);

        body.addForceAtBodyPoint(force, this.centreOfBuoyancy);
    }
};

/**
 * Keeps track of one force generator and the body it applies to.
 */
APE.ForceRegistration = function(body, fg){
    this.body = body;
    this.fg = fg;
};

/**
 * Holds all the force generators and bodies they apply to.
 */
APE.ForceRegistry = function(){
    /**
     * Holds the list of registrations.
     */
    this.registrations = [];
};

APE.ForceRegistry.prototype = {
    constructor: APE.ForceRegistry,
    /**
     * Registers the given force generator to apply to the
     * given body.
     */
    add: function(body, fg){
        var registration = new APE.ForceRegistration(body, fg);
        this.registrations.push(registration);
    },
    /**
     * Removes the given registered pair from the registry.
     * If the pair is not registered, this method will have
     * no effect.
     */
    remove: function(body, fg){
        var registration, found = false;
        for(var i = 0; i < this.registrations.length && !found; i++){
            registration =  this.registrations[i];
            if(registration.body === body && registration.fg === fg){
                this.registrations.splice(i,1);
                found = true;
            }
        }
    },

    /**
     * Clears all registrations from the registry. This will
     * not delete the bodies or the force generators
     * themselves, just the records of their connection.
     */
    clear: function(){
        this.registrations = [];
    },

    /**
     * Calls all the force generators to update the forces
     * of their corresponding bodies.
     */
    updateForces: function(duration){
        var registration;
        for(var i = 0; i < this.registrations.length; i++){
            registration = this.registrations[i];
            registration.fg.updateForce(registration.body, duration);
        }
    }
};
