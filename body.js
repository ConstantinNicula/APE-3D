APE.GRAVITY = new APE.Vector3(0, -9.81, 0);
APE.HIGH_GRAVITY = new APE.Vector3(0, -19.62, 0);
APE.UP = new APE.Vector3(0, 1, 0);
APE.RIGHT = new APE.Vector3(1, 0, 0);
APE.OUT_OF_SCREEN = new APE.Vector3(0, 0, 1);
APE.X = new APE.Vector3(0, 1, 0);
APE.Y = new APE.Vector3(1, 0, 0);
APE.Z = new APE.Vector3(0, 0, 1);

/**
 * Definition of the sleep epsilon.
 */
APE.sleepEpsilon = 0.3;

/**
 * Functions to change the sleep epsilon.
 */
APE.setSleepEpsilon = function(value){
	this.sleepEpsilon = value;
};

APE.getSleepEpsilon = function(){	
	return this.sleepEpsilon;
};

/**
 * Precision values.
 */
APE.real_epsilon = Number.EPSILON;

/**
  * A rigid body is the basic simulation object in the 
  * physics core.
  * 
  * It has position and orientation data, along with first 
  * derivatives. It can be integrated forward through time, 
  * and have forces, torques and impulses (linear or angular) 
  * applied to it. The rigid Body manages its state and allows access
  * through a set of methods.
  *
  */
  
APE.RigidBody = function(){
	/**
	  * This data holds the state of the rigid body. There are two sets
	  * of data: characteristics and state.
	  *
	  * Characteristics are properties of the rigid body
	  * independent of its current kinematic situation. THis 
	  * includes mass, moment of inertia and damping
	  * properties. Two identical bodies will have the same
	  * values for their characteristics.
	  *
	  * State includes all the characteristics and also includes
	  * the kinematic situation of the rigid body in the current
	  * simulation. By setting the whole state data, a rigidBody's 
	  * exact game state can be replicated. Note that state does not
	  * include forces applied to the body. Two identical
	  * rigid bodies in the same simulation will not share the same
	  * state values.
	  *
	  * The state values make up the smallest set of independent 	  
	  * data for the rigid body. Other state data is calculated 
	  * from their current values. When state data is changed the
	  * dependant values need to be updated: this can be achieved 
	  * either by integrating the simulation, or by calling the 
	  * calculateInternals function. THis two stage process is used
	  * because recalculating internals can be a costly process:
	  * all state changes should be carried out at the same time, 
	  * allowing for a single call.
      */

    /**
      * Holds the inverse of the mass of the rigid body. It is more
      * useful to hold the inverse mass because integration is simpler,
      * and because in real time simulation it is more useful to have bodies
      * with infinite mass (immovable) than zero mass
      * (completely unstable in numerical simulation).
    */
	this.inverseMass = 1;
	
	/**
	  * Holds the inverse of the body's inertia tensor. The
	  * inertia tensor must not be degenerate (that would mean 
	  * the body has zero inertia for spinning along one axis).
	  * As long as the tensor is finite, it will be invertible. 
	  * The inverse Tensor is used for similar reasons to the use 
	  * of inverse mass
	  *
	  * The inertia tensor, unlike the other variables that define
	  * a rigid body, is given in body space.
	  */
	this.inverseInertiaTensor = new APE.Matrix3();
	
	/**
	  * Holds the amount of damping applied to linear
	  * motion. Damping is required to remove energy added
	  * through mathematical instability in the integrator.
	  */
	
	this.linearDamping = 0;
	
	/**
	  * Holds the amount of damping applied to angular
	  * motion. Damping is required to remove energy added
	  * through mathematical instability in the integrator.
	  */
	this.angularDamping = 0;

	/**
	  * Holds the linear position of the body in 
	  * world space.
	  */
	this.position = new APE.Vector3();
	
	/**
	  * Holds the angular orientation of the rigid body in 
	  * world space.
	  */
	this.orientation = new APE.Quaternion();
	
	/**
	  * Holds the linear velocity of the rigid body in
	  * world space.
	  */
	this.velocity = new APE.Vector3();
	
	/**
	  * Holds the angular velocity, or rotation, of the
	  * rigid body in world space.
	  */
	this.rotation = new APE.Vector3();
	
	/** 
	  * These members hold information that is derived from
	  * the other data.
	  */
	
	/**
	  * Holds the inverse inertia tensor of the body in the world
	  * space. The inverse inertia tensor member is specified in
	  * the body's local space.
	  */
	this.inverseInertiaTensorWorld = new APE.Matrix3();
	
	/**
	  * Holds the amount of motion of the body. this is a recency
	  * weighted mean that can be used to put a body to sleep.
	  */
	this.motion = 0;
	
	/**
	  * A body can be put to sleep to avoid it being updated
	  * by the integration functions or affected by collisions
	  * with the world.
	  */
	this.isAwake = true;
	
	/**
	  * Some bodies may never be allowed to fall asleep.
	  * User controlled bodies, for example, should be 
	  * always awake.
	  */
	this.canSleep = true;
	
	/**
	  * Holds a transformation matrix for converting body space into
	  * world space  and vice versa. This can be achieved by calling
	  * the getPointIn*Space functions.
	  */
	this.transformMatrix = new APE.Matrix4();

	/**
	  * These data members store the current force, torque and
	  * acceleration of the rigid body. Forces can be added to
	  * the rigid body in any order, and the class decomposes them
	  * into their constituents, accumulating them for the next
	  * simulation step. At the simulation step, the accelerations
	  * are calculated and storred to be applied to the rigid body.
	  */

	/**
	  * Holds the accumulated force to be applied a the nex integration 
	  * step.
	  */
	this.forceAccum = new APE.Vector3();

	/**
	  * Holds the accumulated torque to be applied at the next
	  * integration step.
	  */
	this.torqueAccum = new APE.Vector3();

	
	/**
      * Holds the acceleration of the rigid body. This value
      * can be ised to set acceleration due to gravity (its
      * primary use), or any other constant acceleration.
      */
    this.acceleration = new APE.Vector3();

    /**
      * Holds the linear acceleration of the rigid body, for
      * the previous frame.
      */
     this.lastFrameAccelertaion = new APE.Vector3();
};	



APE.RigidBody.prototype = {
	constructor: APE.RigidBody,
	/** 
	  * Function that creates a transform matrix from a
	  * position and orientation
	  */
	_calculateTransformMatrix: function(transformMatrix, position, orientation){
		transformMatrix.data[0] = 1 - 2 * orientation.j * orientation.j -
		2 * orientation.k * orientation.k;
		transformMatrix.data[1] = 2 * orientation.i * orientation.j -
		2 * orientation.r * orientation.k;
		transformMatrix.data[2] = 2 * orientation.i * orientation.k +
		2 * orientation.r * orientation.j;
		transformMatrix.data[3] = position.x;
		
		transformMatrix.data[4] = 2 * orientation.i * orientation.j +
		2 * orientation.r * orientation.k;
		transformMatrix.data[5] = 1 - 2 * orientation.i * orientation.i -
		2 * orientation.k * orientation.k;
		transformMatrix.data[6] = 2 * orientation.j * orientation.k -
		2 * orientation.r * orientation.i;
		transformMatrix.data[7] = position.y;

		transformMatrix.data[8] = 2 * orientation.i * orientation.k -
		2 * orientation.r * orientation.j;
		transformMatrix.data[9] = 2 * orientation.j * orientation.k +
		2 * orientation.r * orientation.i;
		transformMatrix.data[10] = 1 - 2 * orientation.i * orientation.i -
		2 * orientation.j * orientation.j;
		transformMatrix.data[11] = position.z;
	 
	},
	
	/**
	  * Internal function to do an inertia tensor transform by a quaternion.
	  */
	_transformInertiaTensor: function(iitWorld, q, iitBody, rotmat){
		var t4 = rotmat.data[0]*iitBody.data[0]+
			rotmat.data[1]*iitBody.data[3]+
			rotmat.data[2]*iitBody.data[6];
		var t9 = rotmat.data[0]*iitBody.data[1]+
			rotmat.data[1]*iitBody.data[4]+
			rotmat.data[2]*iitBody.data[7];
		var t14 = rotmat.data[0]*iitBody.data[2]+
			rotmat.data[1]*iitBody.data[5]+
			rotmat.data[2]*iitBody.data[8];
		var t28 = rotmat.data[4]*iitBody.data[0]+
			rotmat.data[5]*iitBody.data[3]+
			rotmat.data[6]*iitBody.data[6];
		var t33 = rotmat.data[4]*iitBody.data[1]+
			rotmat.data[5]*iitBody.data[4]+
			rotmat.data[6]*iitBody.data[7];
		var t38 = rotmat.data[4]*iitBody.data[2]+
			rotmat.data[5]*iitBody.data[5]+
			rotmat.data[6]*iitBody.data[8];
		var t52 = rotmat.data[8]*iitBody.data[0]+
			rotmat.data[9]*iitBody.data[3]+
			rotmat.data[10]*iitBody.data[6];
		var t57 = rotmat.data[8]*iitBody.data[1]+
			rotmat.data[9]*iitBody.data[4]+
			rotmat.data[10]*iitBody.data[7];
		var t62 = rotmat.data[8]*iitBody.data[2]+
			rotmat.data[9]*iitBody.data[5]+
			rotmat.data[10]*iitBody.data[8];

		iitWorld.data[0] = t4*rotmat.data[0]+
			t9*rotmat.data[1]+
			t14*rotmat.data[2];
		iitWorld.data[1] = t4*rotmat.data[4]+
			t9*rotmat.data[5]+
			t14*rotmat.data[6];
		iitWorld.data[2] = t4*rotmat.data[8]+
			t9*rotmat.data[9]+
			t14*rotmat.data[10];
		iitWorld.data[3] = t28*rotmat.data[0]+
			t33*rotmat.data[1]+
			t38*rotmat.data[2];
		iitWorld.data[4] = t28*rotmat.data[4]+
			t33*rotmat.data[5]+
			t38*rotmat.data[6];
		iitWorld.data[5] = t28*rotmat.data[8]+
			t33*rotmat.data[9]+
			t38*rotmat.data[10];
		iitWorld.data[6] = t52*rotmat.data[0]+
			t57*rotmat.data[1]+
			t62*rotmat.data[2];
		iitWorld.data[7] = t52*rotmat.data[4]+
			t57*rotmat.data[5]+
			t62*rotmat.data[6];
		iitWorld.data[8] = t52*rotmat.data[8]+
			t57*rotmat.data[9]+
			t62*rotmat.data[10];
	},
	
	/**
	  * Calculates internal data from state data. This should be called
	  * after the body's state is altered directly (it is called
	  * automatically during integration). If you change the body's state
	  * and then intend to integrate before querying any data (such as the
	  * transform matrix), the  you can omit this step.
	  */
	calculateDerivedData: function(){
		this.orientation.normalize();
		
		//Calculate the transform matrix for the body.
		this._calculateTransformMatrix(this.transformMatrix, this.position, this.orientation);
		
		//Calculate the inertia in world space.
		this._transformInertiaTensor(this.inverseInertiaTensorWorld,
			this.orientation,
			this.inverseInertiaTensor,
			this.transformMatrix);
	},
	
	/**
	  * Integrates the rigid body forward in time by the given amount.
	  * This function uses the Newton-Euler integration method, which is
	  * a linear approximation to the correct integrate. For this reason it
	  * may be inaccurate in some cases.
	  */
    integrate: function(duration){
    if(!this.isAwake){
        return;
    }

    // Calculate linear acceleration from force inputs.
    this.lastFrameAccelertaion = this.acceleration.clone();
    this.lastFrameAccelertaion.addScaledVector(this.forceAccum, this.inverseMass);

    // Calculate angular acceleration from torque inputs.
    var angularAcceleration = this.inverseInertiaTensorWorld.transform(this.torqueAccum);

    // Adjust velocities
    // Update linear velocity from both acceleration and impulse.
    this.velocity.addScaledVector(this.lastFrameAccelertaion, duration);

    // Update angular velocity from both acceleration and impulse.
    this.rotation.addScaledVector(angularAcceleration, duration);

    // Impose drag.
    this.velocity = this.velocity.multiplyScalar(Math.pow(this.linearDamping, duration));
    this.rotation = this.rotation.multiplyScalar(Math.pow(this.angularDamping, duration));

    // Adjust positions.
    // Update linear position.
    this.position.addScaledVector(this.velocity, duration);

    //Update angular position.
    this.orientation.addScaledVector(this.rotation, duration);

    // Normalize the orientation, and update the matrices with new
    // position and orientation
    this.calculateDerivedData();

    // Clear accumulators.
    this.clearAccumulators();

    // Update the kinetic energy store, and possibly put the body to
    // sleep.

    if(this.canSleep){
        var currentMotion = this.velocity.dot(this.velocity) +
            this.rotation.dot(this.rotation);
        var bias = Math.pow(0.5, duration);
        this.motion = bias * this.motion + (1 - bias) * currentMotion;

        if( this.motion < APE.sleepEpsilon){
            this.setAwake(false);
        }else if(this.motion > 10 * APE.sleepEpsilon){
            this.motion = 10 * APE.sleepEpsilon;
        }
    }

    },

	/**
	  * Sets the mass of the rigid body.
	  * The new mass of the body. This may not be zero.
	  * Small masses can produce unstable rigid bodies under
	  * simulation.
	  */
	setMass : function(mass){
		if(mass !== 0){
			this.inverseMass = 1/mass;
		}
	},
	
	/**
	  * Gets the mass of the rigid body.
	  */
	getMass: function(){
		if(this.inverseMass === 0){
			return Number.MAX_VALUE;
		}else{
			return 1/this.inverseMass;
		}
	},
	
	/**
	  * Sets the inverse mass of the rigid body.
	  * The new inverse mass of the body may be zero in
	  * the case of bodies with infinite mass(unmovable).
	  */
	setInverseMass: function(inverseMass){
		this.inverseMass = inverseMass; 
	},
	
	/**
	  * Gets the inverse mass of the rigid body.
	  */
	getInverseMass :function(){
		return this.inverseMass;
	},
	
	/**
	  * Returns true if the mass of the body is non-infinite.
	  */
	hasFiniteMass : function(){
		return this.inverseMass >= 0;
	},

	/**
	  * Sets the inertia tensor for the rigid body.
	  * The inertia tensor for the rigid body must be a full
	  * rank matrix and must be invertible.
	  */
	setInertiaTensor: function(inertiaTensor){
		this.inverseInertiaTensor.setInverse(inertiaTensor);
	},
	
	
	/**
	  * Gets a copy of the current inertia tensor of the rigid body.
	  * Returns a new matrix containing the current inertia 
	  * tensor. The inertia tensor is expressed in the rigid body's
	  * local space. 
	  */
	getInertiaTensor: function(){
		var inertiaTensor = new APE.Matrix3();
		inertiaTensor.setInverse(this.inverseInertiaTensor);
		return inertiaTensor;
	},

	/**
	  * Gets a copy of the current inertia tensor of the rigid
	  * body. A new matrix containing the current inertia tensor
	  * is returned. The inertia tensor is expressed in world 
	  * space. 
	  */
	getInertiaTensorWorld : function(){
		var inertiaTensorWorld = new APE.Matrix3();
		inertiaTensorWorld.setInverse(this.inverseInertiaTensorWorld);
		return inertiaTensorWorld;
	},

	/**
	  * Sets the inverse inertia tensor for the rigid body.
	  * The inverse inertia tensor for the rigid body must be a 
	  * full rank matrix and must be invertible.
	  */
	setInverseInertiaTensor: function(inverseInertiaTensor){
		this.inverseInertiaTensor = inverseInertiaTensor.clone();
	},

	/**
	  * Gets a copy of the current inverse inertia tenosr of the
	  * rigid body.
	  * A new matrix is returned containing the current inverse 
	  * inertia tensor. The inertia tensor is expressed in the
	  * rigid body's local space.
	  */
	getInverseInertiaTensor: function(){
		return this.inverseInertiaTensor.clone();
	},

	/**
	  * Gets a copy of the current inverse inertia tensor of the rigid
	  * body.
	  * A new matrix is returned containing the current inverse 
	  * inertia tensor. The inertia tensor is expressed in world
	  * space.
	  */
	getInverseInertiaTensorWorld: function(){
		return this.inverseInertiaTensorWorld.clone();	
	},

	/**
	  * Sets both linear and angular damping in one function call
	  * Linear damping is the speed that that is shed from the 
	  * velocity of the rigid body.
	  * Angular damping is the rotation speed that is shed from the
	  * rigid body.
	  */
	setDamping: function(linearDamping, angularDamping){
		this.linearDamping = linearDamping;
		this.angularDamping = angularDamping;
	},

	/**
	  * Sets the linear damping for the rigid body.
	  */
	setLinearDamping: function(linearDamping){	
		this.linearDamping = linearDamping;
	},

	/**
	  * Gets the current linear damping value.
	  */
	getLinearDamping: function(){
		return this.linearDamping;
	},

	/**
	  * Sets the angular damping for the rigid body.
	  */
	setAngularDamping: function(angularDamping){
		this.angularDamping = angularDamping;
	},

	/**
	  * Gets the current angular damping value.
	  */
	getAngularDamping: function(){
		return this.angularDamping;
	},

	/**
	  * Sets the position of the rigid body.
	  */
	setPosition: function(position){
		this.position.x = position.x;
		this.position.y = position.y;
		this.position.z = position.z;
	},

	/**
	  * Gets the position of the rigid body.
	  */
	getPosition: function(){
		return this.position.clone();
	},

	/**
	  * Sets the orientation of the rigid body.
	  * The given orientation does not need to be
	  * normalized, and can be zero. This function
	  * automatically constructs a valid rotation quaternion
	  * with (0, 0, 0, 0) mapped to (1, 0, 0, 0)
	  */
	setOrientation: function(orientation){
		this.orientation.r = orientation.r;
		this.orientation.i = orientation.i;
		this.orientation.j = orientation.j;
		this.orientation.k = orientation.k;
		this.orientation.normalize();
	},

	/**
	  * Gets the orientation of the rigid body as a quaternion.
	  */
	getOrientation: function(){
		return this.orientation.clone();
	},

	/**
	  * Gets the orientation of the rigid body as a matrix with
	  * a transformation representing the rigid body's orientation.
	  */
	getOrientationMatrix: function(){
		var orientation = new APE.Matrix3();
		orientation.data[0] = this.transformMatrix.data[0];
		orientation.data[1] = this.transformMatrix.data[1];
		orientation.data[2] = this.transformMatrix.data[2];
		
		orientation.data[3] = this.transformMatrix.data[4];
		orientation.data[4] = this.transformMatrix.data[5];
		orientation.data[5] = this.transformMatrix.data[6];
		
		orientation.data[6] = this.transformMatrix.data[8];
		orientation.data[7] = this.transformMatrix.data[9];
		orientation.data[8] = this.transformMatrix.data[10];
		return orientation;
	},

	/**
	  * Gets a transformation matrix representing the 
	  * rigid body's position and orientation. Transforming
	  * a vector by this matrix turns it from the body's local
	  * space to world space.
	  */
	getTransform: function(){
		return this.transformMatrix.clone();
	},

	/**
	  * Converts the given point from world space into the body's
	  * local space.
	  */
	getPoinInLocalSpace: function(point){
		return this.transformMatrix.transformInverse(point);
	},

	/**
	  * Converts the given point from local sapce into the 
	  * world space.
	  */
	getPointInWorldSpace: function(point){
		return this.transformMatrix.transform(point);
	},

	/** 
	  * Converst the given direction from world space into 
	  * the body's local space.
	  *
	  * When a direction is converted between frmes of 
	  * reference, there is no translation required.
	  */
	getDirectionInLocalSpace: function(direction){
		return this.transformMatrix.transformInverseDirection(direction);
	},

	/**
	  * Converts the given direction from world space into the
	  * body's local space.
	  * 
	  * When a direction is converted between frames of reference,
	  * there is no translation required.
	  */
	 getDirectionInWorldSpace: function(direction){
	 	return this.transformMatrix.transformDirection(direction);
	 },

	/**
	  * Sets the velocity of the rigid body.
	  */
	setVelocity: function(velocity){
		this.velocity.x = velocity.x;
		this.velocity.y = velocity.y;
		this.velocity.z = velocity.z;	
	},

	/**
	  * Gets the velocity of the rigid body.
	  */
	getVelocity: function(){
		return this.velocity.clone();
	},

	/**
	  * Applies the given change in velocity/
	  */
	addVelocity: function(deltaVelocity){
		this.velocity = this.velocity.add(deltaVelocity);
	},

	/**
	  * Sets the rotation of the rigid body. The rotation is
	  * given in world space.
	  */
	setRotation: function(rotation){
		this.rotation.x = rotation.x;
		this.rotation.y = rotation.y;
		this.rotation.z = rotation.z;
	},

	/**
	  * Gets the rotation of the rigid body. The rotation is 
	  * given in world local space.  
	  */
	getRotation: function(){
		return this.rotation.clone();
	},

	/**
	  * Applies the given change in rotation.
	  */
	addRotation: function(deltaRotation){
		this.rotation = this.rotation.add(deltaRotation);
	},

	/** 
	  * Returns true if the body is awake and responding to 
	  * integration.
	  */
	getAwake: function(){
		return this.isAwake;
	},

	/**
	  * Sets the awake state of the body. If the body is set to be
	  * not awake, then its velocities are also cancelled, since 
	  * a moving body that is not awake can cause problems in the
	  * simulation.
	  */
	setAwake: function(awake){
	 	if(awake){
	 		this.isAwake = true;
	 		// Add a bit of motion to avoid it falling asleep immediately.
	 		this.motion = APE.sleepEpsilon * 2;
	 	} else{
	 		this.isAwake = false;
	 		this.velocity.clear();
	 		this.rotation.clear();
	 	}
	},

	/**
	  * Returns true if the body is allowed to go to sleep at
	  * any time.
	  */
	getCanSleep: function(){
		return this.canSleep;
	},

	/**
	  * Sets wether the body si ever allowed to go to sleep.
	  * Bodies under the player's control, or for which the set of 
	  * transient forces applied each frame are not predictable,
	  * shold be kept awake.
	  */
	setCanSleep: function(canSleep){
		this.canSleep = canSleep;
		if(!this.canSleep  && !this.isAwake){
			this.setAwake(true);
		}
	},

	/**
	  * These functions provide access to the acceleration properties
	  * of the body. The acceleration is generated by the simulation 
	  * from the forces and torques applied to the rigid bod. 
	  * Acceleration cannot be directly influenced, it is set during the integration,
	  * and represent the acceleration experienced by the body of the
	  * previous simulation step.
	  */

	/**
	  * Gets the current accumulated value for the linear acceleration.
	  * The acceleration accumulators are set during the integration step.
	  * They can be read to determine the rigid body's acceleration 
	  * over the last integration step. The linear acceleratio is 
	  * given in world space. 
	  */

	getLastFrameAcceleration: function(){
		return this.lastFrameAccelertaion.clone();
	},
	  

	/**
	  * Force, Torque and Acceleration Set-up Functions
	  *
	  * These functions set up forces and torques to apply to the
	  * rigid body.
	  */
	
	/**
	  * Clears the forces and torques in the accumulators. This will
	  * be called automatically after each intergartion step.
	  */
	clearAccumulators : function(){
		this.forceAccum.clear();
		this.torqueAccum.clear();
	},

	/**
	  * Adds the given force to the centre of mass of the rigid body.
	  * The force is expressed in world coordinates.
	  */
	addForce : function(force){
		this.forceAccum = this.forceAccum.add(force);
		this.isAwake =  true;
	},

	
	/**
	  * Adds the given force to the given point on the rigid body.
	  * Both the force and the application point are given in world space.
	  * Because the force is not applied at the centre of mass, it may be
	  * split into both a force and a torque.
	  */
	addForceAtPoint: function(force, point){
		// Convert to coordinates relative to the centre of mass.
		var pt = point.clone();
		pt = pt.sub(this.position);
		
		this.forceAccum = this.forceAccum.add(force);
		this.torqueAccum = this.torqueAccum.add(pt.cross(force));

		this.isAwake = true;
	},
	
	/**
	  * Adds the given force to the given point on the rigid body.
	  * The direction of the force is given in world coordinates,
	  * but the application point is given in body space. This is
	  * useful for spring forces, or other forces fixed to the body.
	  */
	addForceAtBodyPoint: function(force, point){
		// Convert to coordinates relative to centre of mass.
		var pt = this.getPointInWorldSpace(point);
		this.addForceAtPoint(force, pt);
	},

	/** 
	  * Adds the given torque to the rigid body.
	  * The force is expressed in world-coordinates.
	  */
	addTorque: function(torque){
		this.torqueAccum = this.torqueAccum.add(torque);
		this.isAwake = true;
	},

	/** 
	  * Sets the constant acceeration of the rigid body.
	  */
	setAcceleration: function(acceeration){
		this.acceleration.x = acceeration.x;
		this.acceleration.y = acceeration.y;
		this.acceleration.z = acceeration.z;
	},

	/**
	  * Gets the acceleration of the rigid body.
	  */
	getAcceleration: function(){
		return this.acceleration.clone();
	}
};

