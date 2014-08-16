/**
 * This file contains the contact resolution system, although it
 * is called the contact resolution system, it handles collisions, contacts
 * (sliding and resting), and constraints (such as joints).
 *
 * The resolver uses an iterative satisfaction algorithm; it loops
 * through each contact and tries to resolve it. This is a very fast
 * algorithm but can be unstable when the contacts are highly inter- related.
 */



/**
 * A contact represents two bodies in contact. Resolving a
 * contact removes their interpenetration, and applies sufficient
 * impulse to keep them apart. Colliding bodies may also rebound.
 * Contacts can be used to represent positional joints, by making
 * the contact constraint keep the bodies in their correct
 * orientation.
 *
 * It can be a good idea to create a contact object even when the
 * contact isn't violated. Because resolving one contact can violate
 * another, contacts that are close to being violated should be
 * sent to the resolver; that way if one resolution moves the body,
 * the contact may be violated, and can be resolved. If the contact
 * is not violated, it will not be resolved, so you only loose a
 * small amount of execution time.
 *
 * The contact has no callable function, it just holds the contact
 * details. To resolve a set of contacts, use the contact resolver
 * class.
 */

APE.Contact = function(){
    /**
     * Holds the bodies that are involved in the contact. The
     * second of these can be null, for contacts with the scenery.
     */
    this.body = [];

    /**
     * Holds the lateral friction coefficient at the contact.
     */
    this.friction = 0;

    /**
     * Holds the normal restitution coefficient at the contact.
     */
    this.restitution = 0;

    /**
     * Holds the position of the contact in world coordinates.
     */
    this.contactPoint = new APE.Vector3();

    /**
     * Holds the direction of the contact in world coordinates.
     */
    this.contactNormal = new APE.Vector3();

    /**
     * Holds the depth of penetration at the contact point. If both
     * bodies are specified the the contact point should be midway
     * between the inter-penetrating points.
     */
    this.penetration = 0;

    /**
     * A transform matrix that converts co-ordinates in the contact's
     * frame of reference to the world co-ordinates. The columns of this
     * matrix form an orthonormal set of vectors.
     */
    this.contactToWorld = new APE.Matrix3();

    /**
     * Holds the closing velocity at the point of contact. This is set
     * when the calculateInternals function is run.
     */
    this.contactVelocity = new APE.Vector3();

    /**
     * Holds the closing velocity at the point of contact. This is set
     * when the calculateInternals function is run.
     */
    this.desiredDeltaVelocity = 0;

    /**
     * Holds the world space position of the contact point relative to
     * the centre of each body. This is set when the calculateInternals
     * function is run.
     */
    this.relativeContactPosition = [];
};

APE.Contact.prototype = {
    constructor: APE.Contact,

    /**
     * Sets the data that doesn't normally depend on the position
     * of the contact(the bodies, and their material properties).
     */
    setBodyData: function(one, two, friction , restitution){
        this.body[0] = one;
        this.body[1] = two;

        this.friction = friction;
        this.restitution = restitution;
    },

    /**
     * Calculates an orthonormal basis for the contact point, based on
     * the primary friction direction(for anisotropic friction) or a
     * random orientation (for isotropic friction).
     *
     * Constructs an arbitrary orthonormal basis for the contact. This
     * is stored as a 3x3 matrix where each vector is a column (in other
     * words the matrix transforms contact space into world space). The
     * x direction is generated from the contact normal and the y and z
     * directions are set so they ar at right angles to it.
     */
    calculateContactBasis: function(){
        var contactTangent = [];
        // Check whether the Z-axis is nearer to the X or Y axis.
        if(Math.abs(this.contactNormal.x) > Math.abs(this.contactNormal.y)){
            // Scaling factor to ensure the results are normalised
            var s = 1/Math.sqrt(this.contactNormal.z * this.contactNormal.z +
                this.contactNormal.x * this.contactNormal.x);

            // The new X-axis is at right angles to the world Y axis
            contactTangent[0] = new APE.Vector3();
            contactTangent[0].x = this.contactNormal.z * s;
            contactTangent[0].y = 0;
            contactTangent[0].z = -this.contactNormal.x * s;

            // The new Y-axis is at right angles to the new X and Z axes
            contactTangent[1] = new APE.Vector3();
            contactTangent[1].x = this.contactNormal.y * contactTangent[0].x;
            contactTangent[1].y = this.contactNormal.z * contactTangent[0].x -
                this.contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -this.contactNormal.y * contactTangent[0].x;
        }else{
            // Scaling factor to ensure the results are normalised
            var s = 1/Math.sqrt(this.contactNormal.z * this.contactNormal.z +
                this.contactNormal.y * this.contactNormal.y);

            // The new X-ais is at right angles to thw world X-axis
            contactTangent[0] = new APE.Vector3();
            contactTangent[0].x = 0;
            contactTangent[0].y = -this.contactNormal.z * s;
            contactTangent[0].z = this.contactNormal.y * s;

            // The new Y-axis is at right angles to the new X and Z axes.
            contactTangent[1] = new APE.Vector3();
            contactTangent[1].x = this.contactNormal.y * contactTangent[0].z -
                this.contactNormal.z * contactTangent[0].y;
            contactTangent[1].y = -this.contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = this.contactNormal.x * contactTangent[0].y;
        }

        // Make a matrix from the three vectors.
        this.contactToWorld.setComponents(
            this.contactNormal,
            contactTangent[0],
            contactTangent[1]
        );
    },

    /**
     * Updates the awake state of rigid bodies that are taking
     * place int the given contact. A body will be made awake if it
     * is in contact with a body that is awake.
     */
    matchAwakeState: function(){
        // Collisions with the world never cause a body to wake up.
        if(!this.body[1]){
            return;
        }
        var body0awake = this.body[0].getAwake();
        var body1awake = this.body[1].getAwake();

        // Wake up only the sleeping one
        if(body0awake ^ body1awake){
            if(body0awake){
                this.body[1].setAwake(true);
            }else{
                this.body[0].setAwake(true);
            }
        }
    },

    /**
     * Reverse the contact. This involves swapping the two rigid bodies
     * and reversing the contact normal. The internal values should then
     * be recalculated using calculateInternals( this is not do it
     * automatically).
     *
     * Swaps the bodies in the current contact, so body 0 is at body 1 and
     * vice versa. This also changes the direction of the contact normal,
     * but doesn't update any calculated internal data. If you are calling
     * this method manually, then call calculateInternal afterwards to make
     * sure the internal data is up to date.
     */
    swapBodies: function(){
        this.contactNormal.invert();

        var temp = this.body[0];
        this.body[0] = this.body[1];
        this.body[1] = temp;
    },

    /**
     * Calculates and returns the velocity of the contact point on
     * the given body.
     */
    calculateLocalVelocity: function(bodyIndex, duration){
       var thisBody = this.body[bodyIndex];

        // Work out the velocity of the contact point.
        var velocity = thisBody.getRotation().cross(this.relativeContactPosition[bodyIndex]);
        velocity = velocity.add(thisBody.getVelocity());

        // Turn the velocity into contact-coordinates.
        var contactVelocity = this.contactToWorld.transformTranspose(velocity);

        // Calculates the amount of velocity that is due to forces without
        // reactions.
        var accVelocity = thisBody.getLastFrameAcceleration().multiplyScalar(duration);

        // Calculate the velocity in contact-coordinates.
        accVelocity = this.contactToWorld.transformTranspose(accVelocity);

        // We ignore any component of acceleration in the contact normal direction
        // we are only interested in planar acceleration.
        accVelocity.x = 0;

        // Add the planar velocities - if there's enough friction they will
        // be removed during velocity resolution.
        contactVelocity = contactVelocity.add(accVelocity);

        // And return it.
        return contactVelocity;
    },

    /**
     * Calculates and sets the internal value of the desired delta
     * velocity.
     */
    calculateDesiredDeltaVelocity: function(duration){
        var velocityLimit = 0.25;

        // Calculate the acceleration induced velocity accumulated this frame.
        var velocityFromAcc = 0;
        if(this.body[0].getAwake()){
            velocityFromAcc +=
                this.body[0].getLastFrameAcceleration().multiplyScalar(duration).dot(this.contactNormal);
        }

        if(this.body[1] && this.body[1].getAwake()){
            velocityFromAcc -=
                this.body[1].getLastFrameAcceleration().multiplyScalar(duration).dot(this.contactNormal);
        }

        // If the velocity is very slow, limit the restitution
        var thisRestitution = this.restitution;
        if(Math.abs(this.contactVelocity.x) < velocityLimit){
            thisRestitution = 0;
        }

        // Combine bounce velocity with the removed
        // acceleration velocity.

        this.desiredDeltaVelocity =
            -this.contactVelocity.x
            -thisRestitution * (this.contactVelocity.x - velocityFromAcc);
    },

    /**
     * Calculates internal data from state data. This is called before
     * the resolution algorithm tries to do any resolution. It should
     * never need to be called manually.
     */
    calculateInternals: function(duration){
        // Check if the first object is null, and swap if it is.
        if(!this.body[0]){
            this.swapBodies();
        }

        // Calculate an set of axis at the contact point.
        this.calculateContactBasis();

        // Store the relative position of the contact relative to each body.
        this.relativeContactPosition[0] = this.contactPoint.sub(this.body[0].getPosition());
        if(this.body[1]){
            this.relativeContactPosition[1] = this.contactPoint.sub(this.body[1].getPosition());
        }

        // Find the relative velocity of the bodies at the contact point.
        this.contactVelocity = this.calculateLocalVelocity(0, duration);
        if(this.body[1]){
            this.contactVelocity =  this.contactVelocity.sub(this.calculateLocalVelocity(1, duration));
        }

        // Calculate the desired change in velocity for resolution
        this.calculateDesiredDeltaVelocity(duration);
    },

    /**
     * Performs an inertia-weighted impulse based resolution of this
     * contact alone.
     */
    applyVelocityChange: function(velocityChange, rotationChange){
        // Get hold of the inverse mass and inverse inertia tensor, both in
        // world coordinates.
        var inverseInertiaTensor = [];
        inverseInertiaTensor[0] = this.body[0].getInverseInertiaTensorWorld();
        if(this.body[1]){
            inverseInertiaTensor[1] = this.body[1].getInverseInertiaTensorWorld();
        }

        // We will calculate the impulse for each contact axis.
        var impulseContact;

        if(this.friction === 0){
            // Use the short format for frictionless contacts
            impulseContact = this.calculateFrictionlessImpulse(inverseInertiaTensor);
        }else
        {
            // Otherwise we may have impulses that aren't in the direction
            // of the contact, so we need the more complex version.
            impulseContact = this.calculateFrictionImpulse(inverseInertiaTensor);
        }

        // Convert the impulse to world coordinates
        var impulse = this.contactToWorld.transform(impulseContact);

        // Split the input into liner and rotational components
        var impulsiveTorque = this.relativeContactPosition[0].cross(impulse);
        rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
        velocityChange[0] = new APE.Vector3();
        velocityChange[0].addScaledVector(impulse, this.body[0].getInverseMass());

        // Apply the changes
        this.body[0].addVelocity(velocityChange[0]);
        this.body[0].addRotation(rotationChange[0]);

        if(this.body[1]){
            // Work out body one's liner and angular changes
            impulsiveTorque = impulse.cross(this.relativeContactPosition[1]);
            rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
            velocityChange[1] = new APE.Vector3();
            velocityChange[1].addScaledVector(impulse, -this.body[1].getInverseMass());

            // And apply them
            this.body[1].addVelocity(velocityChange[1]);
            this.body[1].addRotation(rotationChange[1]);
        }
    },

    /**
     * Calculates the impulse needed to resolve this contact,
     * given that the contact has no friction. A pair of inertia
     * tensors - one for each contact object - is specified to
     * save calculation time: the calling function has access to
     * these anyway
     */
    calculateFrictionlessImpulse: function(inverseInertiaTensor){
        var impulseContact = new APE.Vector3();

        // Build a vector that shows the change in velocity in
        // world space for unit impulse in the direction of the contact
        // normal.
        var deltaVelWorld = this.relativeContactPosition[0].cross(this.contactNormal);
        deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
        deltaVelWorld = deltaVelWorld.cross(this.relativeContactPosition[0]);

        // Work out the change in velocity in contact coordinates.
        var deltaVelocity = deltaVelWorld.dot(this.contactNormal);

        // Add the linear component of velocity change.
        deltaVelocity += this.body[0].getInverseMass();

        // Check if we need to do the second body's data
        if(this.body[1]){
            deltaVelWorld = this.relativeContactPosition[1].cross(this.contactNormal);
            deltaVelWorld= inverseInertiaTensor[1].transform(deltaVelWorld);
            deltaVelWorld = deltaVelWorld.cross(this.relativeContactPosition[1]);

            // Add the change in velocity due to rotation
            deltaVelocity += deltaVelWorld.dot(this.contactNormal);

            // Add the change in velocity due to linear motion
            deltaVelocity += this.body[1].getInverseMass();
        }

        // Calculate the required size of the impulse
        impulseContact.x = this.desiredDeltaVelocity / deltaVelocity;
        impulseContact.y = 0;
        impulseContact.z = 0;
        return impulseContact;

    },

    /**
     * Calculates the impulse needed to resolve the contact,
     * given that the contact has a non-zero coefficient of
     * friction. A pair of inertia tensors - one for each contact
     * object - is specified to save calculation time: the calling
     * function has access to these anyway.
     */
    calculateFrictionImpulse: function(inverseInertiaTensor){
        var impulseContact;
        var inverseMass = this.body[0].getInverseMass();

        // The equivalent of a cross product in matrices is multiplication
        // by a skew symmetric matrix - we build the matrix for converting
        // between linear  and angular quantities.
        var impulseToTorque = new APE.Matrix3();
        impulseToTorque.setSkewSymmetric(this.relativeContactPosition[0]);

        // Build the matrix to convert contact impulse to change in velocity
        // in world coordinates.
        var deltaVelWorld = impulseToTorque.clone();
        deltaVelWorld = deltaVelWorld.multiply(inverseInertiaTensor[0]);
        deltaVelWorld = deltaVelWorld.multiply(impulseToTorque);
        deltaVelWorld = deltaVelWorld.multiplyScalar(-1);

        // Check if we need to add body two's data
        if(this.body[1]){
            // Set the cross product matrix
            impulseToTorque.setSkewSymmetric(this.relativeContactPosition[1]);

            // Calculate the velocity change matrix
            var deltaVelWorld2 = impulseToTorque.clone();
            deltaVelWorld2 = deltaVelWorld2.multiply(inverseInertiaTensor[1]);
            deltaVelWorld2 = deltaVelWorld2.multiply(impulseToTorque);
            deltaVelWorld2 = deltaVelWorld2.multiplyScalar(-1);

            // Add to the total delta velocity.
            deltaVelWorld = deltaVelWorld.add(deltaVelWorld2);

            // Add to the inverse mass
            inverseMass += this.body[1].getInverseMass();
        }


        // Do a change of basis to convert into contact coordinates.
        var deltaVelocity = this.contactToWorld.transpose();
        deltaVelocity = deltaVelocity.multiply(deltaVelWorld);
        deltaVelocity = deltaVelocity.multiply(this.contactToWorld);

        // Add in the linear velocity change
        deltaVelocity.data[0] += inverseMass;
        deltaVelocity.data[4] += inverseMass;
        deltaVelocity.data[8] += inverseMass;

        // Invert to get the impulse needed per unit velocity
        var impulseMatrix = deltaVelocity.inverse();

        // Find the target velocities to kill
        var velKill = new APE.Vector3(this.desiredDeltaVelocity,
            -this.contactVelocity.y,
            -this.contactVelocity.z);

        // Find the impulse to kill target velocities.
        impulseContact = impulseMatrix.transform(velKill);

        // Check for exceeding friction
        var planarImpulse = Math.sqrt(
            impulseContact.y * impulseContact.y +
            impulseContact.z * impulseContact.z
        );
        if(planarImpulse > impulseContact.x * this.friction){
            // We need to use dynamic friction
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;

            impulseContact.x = deltaVelocity.data[0]+
                deltaVelocity.data[1] * this.friction * impulseContact.y +
                deltaVelocity.data[2] * this.friction * impulseContact.z;
            impulseContact.x = this.desiredDeltaVelocity/ impulseContact.x;
            impulseContact.y *= this.friction * impulseContact.x;
            impulseContact.z *= this.friction * impulseContact.x;
        }
        return impulseContact;
    },

    /**
     * Performs an inertia weighted penetration resolution of this
     * contact alone.
     */
    applyPositionChange: function(linearChange, angularChange, penetration){
        var angularLimit = 0.2,
            angularMove = [],
            linearMove = [],

            totalInertia = 0,
            linearInertia = [],
            angularInertia = [];

        // We need to work out the inertia of each object in the direction
        // of the contact normal, due to angular Inertia only.
        for(var i = 0; i <2; i++){
            if(this.body[i]) {
                var inverseInertiaTensor = this.body[i].getInverseInertiaTensorWorld();

                // We use the same procedure as for calculating frictionless
                // velocity change to work out the angular inertia.
                var angularInertiaWorld = this.relativeContactPosition[i].cross(this.contactNormal);
                angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
                angularInertiaWorld = angularInertiaWorld.cross(this.relativeContactPosition[i]);
                angularInertia[i] = angularInertiaWorld.dot(this.contactNormal);
                // The linear component is simply the inverse mass
                linearInertia[i] = this.body[i].getInverseMass();
                // Keep track of the total inertia from all components
                totalInertia += linearInertia[i] + angularInertia[i];
                // We break the loop here so that the total inertia value is
                // completely calculated (for both iterations) before continuing.
            }
        }

        for(var  i = 0; i<2; i++){
            if(this.body[i]){
                // The linear and angular movements are in proportion to
                // the two inverse inertia's.
                var sign = (i === 0)? 1 : -1;
                angularMove[i] =
                    sign * penetration * (angularInertia[i] / totalInertia);
                linearMove[i] =
                    sign * penetration * (linearInertia[i] / totalInertia);

                // To avoid angular projections that are too grate (when mass is
                // large but inertia tensor is small) limit the angular move.
                var projection = this.relativeContactPosition[i].clone();
                projection.addScaledVector(
                    this.contactNormal,
                    -this.relativeContactPosition[i].dot(this.contactNormal)
                );

                // Use the small angle approximation for sine of the angle
                // the magnitude would be sine(angularLimit) * projection.magnitude
                // but we approximate sine(angularLimit) to angularLimit.
                var maxMagnitude = angularLimit * projection.magnitude();

                if(angularMove[i] < - maxMagnitude){
                    var totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = -maxMagnitude;
                    linearMove[i] =  totalMove - angularMove[i];
                }else if(angularMove[i] > maxMagnitude){
                    var totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }


                // We have the linear amount of movement required by turning
                // the rigid body (in angularMove[i]). We now need to
                // calculate the desired rotation to achieve that.
                if(angularMove[i] === 0){
                    // Easy case - no angular movement means no rotation.
                    angularChange[i] = new APE.Vector3();
                }else {
                    // Work out the direction we'd like to rotate in.
                    var targetAngularDirection =
                        this.relativeContactPosition[i].cross(this.contactNormal);

                    var inverseInertiaTensor = this.body[i].getInverseInertiaTensorWorld();

                    // Work out the direction we'd need to rotate to achieve that
                    angularChange[i] = inverseInertiaTensor.transform(targetAngularDirection)
                    angularChange[i] = angularChange[i].multiplyScalar(angularMove[i] / angularInertia[i]);
                }

                //Velocity change is easier -it is just the linear movement
                // along the contact normal.
                linearChange[i] = this.contactNormal.multiplyScalar(linearMove[i]);
                // Now we can start to apply the values we've calculated.
                // Apply the linear movement.
                var pos = this.body[i].getPosition();
                pos.addScaledVector(this.contactNormal, linearMove[i]);
                this.body[i].setPosition(pos);

                // And the change in orientation
                var q = this.body[i].getOrientation();
                q.addScaledVector(angularChange[i], 1);
                this.body[i].setOrientation(q);

                // We need to calculate the derived data for any body that is
                // asleep, so that the changes are reflected in the object's
                // data. Otherwise the resolution will not change the position
                // of the object, and the next collision detection round will
                // have the same penetration.

                if(!this.body[i].getAwake()){
                    this.body[i].calculateDerivedData();
                }
            }
        }
    }
};

/**
 * The contact resolution routine. One resolver instance can be shared
 * for the whole simulation, as long as you need roughly the same
 * parameters each time (which is normal).
 *
 * The resolver uses an iterative satisfaction algorithm; it loops
 * through each contact and tries to resolve it. Each contact is
 * resolved locally, which may in turn put other contacts in a worse
 * position. The algorithm then revisits other contacts and repeats
 * the process up to a specified iteration limit. It can be proved that
 * the given enough iterations, the simulation will get to the correct
 * result. As with all approaches, numerical stability can cause problems
 * that make a correct resolution impossible.
 *
 * This algorithm is very fast, much faster tha other physics approaches.
 * Even when using many more iterations than there are contacts, it will
 * be faster tha global approaches.
 *
 * Many global algorithms are unstable under high friction, this
 * approach is very robust indeed for high friction and low
 * restitution values.
 *
 * The algorithm produces visually believable behaviour. Trade-offs
 * have been mad to err on the side of visual realism rather than
 * computational expense or numerical accuracy.
 *
 * The algorithm does not cope well with situations with many inter-related
 * contacts: stacked boxes, for example. In this
 * case the simulation may appear to jiggle slightly, which often dislodges
 * a box from the stack, allowing it to collapse.
 *
 * Another issue with the resolution mechanism is that resolving one contact
 * may make another contact move sideways against friction, because each contact is
 * handled independently, this friction is not taken into account. If one object is
 * pushing against another, the pushed object may move across its support
 * without friction, even though friction is set between those bodies,
 *
 * In general this resolver is not suitable for stacks of bodies,
 * but is perfect for handling impact, explosive, and flat resting
 * situations.
 */

APE.ContactResolver = function(velocityIterations, positionIterations, velocityEpsilon, positionEpsilon){
    /**
     * Holds the number of iterations to perform when resolving
     * velocity.
     */
    this.velocityIterations = velocityIterations;

    /**
     * Holds the number of iterations to perform when resolving
     * position.
     */
    this.positionIterations = positionIterations;

    /**
     * To avoid instability velocities smaller than this value are considered
     * to be zero. Too small and the simulation may be unstable, too large and
     * the bodies may interpenetrate visually. A good starting point is the
     * default 0.01.
     */
    this.velocityEpsilon = (velocityEpsilon !== undefined)? velocityEpsilon : 0.01;

    /**
     * To avoid instability penetrations smaller than this value are considered
     * to not be interpenetrating. Too small and the simulation may be unstable,
     * too large and the bodies may interpenetrate visually. A good starting point is
     * the default 0.01.
     */
    this.positionEpsilon =(positionEpsilon !== undefined)? positionEpsilon : 0.01;

    /**
     * Stores the number of velocity iterations used on the last call
     * to resolve contacts.
     */
    this.velocityIterationsUsed = 0;

    /**
     * Stores the number of position iterations used in the last call to
     * resolve contacts.
     */
    this.positionIterationsUsed = 0;

    /**
     * Keeps track of whether the internal settings are valid.
     */
    this.validSettings = 0;
};

APE.ContactResolver.prototype = {
    constructor: APE.ContactResolver,
    /**
     * Returns true if the resolver has valid settings and is ready to go.
     */
    isValid : function(){
        return(this.velocityIterations > 0) &&
            (this.positionIterations > 0) &&
            (this.velocityEpsilon >= 0.01) &&
            (this.positionEpsilon >= 0.01);
    },

    /**
     * Set the number of iterations for each resolution stage.
     */
    setIterations: function(velocityIterations, positionIterations){
        this.velocityIterations = velocityIterations;
        this.positionIterations = positionIterations;
    },

    /**
     * Set the tolerance value for both velocity and position.
     */
    setEpsilon: function(velocityEpsilon, positionEpsilon){
        this.velocityEpsilon = velocityEpsilon;
        this.positionEpsilon = positionEpsilon;
    },

    /**
     * Resolves a set of contacts for both penetration and velocity.
     *
     * Contacts that cannot interact with each other should be passed to separate
     * calls to resolveContacts, as the resolution algorithm takes much longer
     * for lots of contacts than it does for the same number of contacts in small
     * sets.
     *
     * Pointer to an array of contact objects. The number of contacts in the array
     * to resolve.
     *
     * The number of iterations through the resolution algorithm. This should be
     * at least the number of contacts (otherwise some constraints will not be
     * resolved - although sometimes this is not noticeable). If the iterations
     * are not needed they will not be used, so adding more iterations may
     * not make any difference. In some cases you would need millions of iterations.
     * Think about the number of iterations  as a bound: if you specify a large number,
     * sometimes the algorithm will use it and you may drop lots of frames.
     */

    resolveContacts: function(contacts, numContacts, duration){
        // Make sure we have something to do.
        if(numContacts === 0){
            return;
        }
        if(!this.isValid()){
            return;
        }
        // Prepare the contacts for processing
        this.prepareContacts(contacts, numContacts, duration);

        // Resolve the interpenetration problems with the contacts.
        this.adjustPosition(contacts, numContacts, duration);

        // Resolve the velocity problems with the contacts.
        this.adjustVelocities(contacts, numContacts, duration);
    },

    /**
     * Sets up contacts ready for processing. This males sure that their internal
     * data is configured correctly and the correct set of bodies is made alive.
     */
    prepareContacts: function(contacts, numContacts, duration){
        // Generate contact velocity and axis information.
        for(var i = 0; i < contacts.length; i++){
            // Calculate the internal contact data(inertia, basis, etc).
            contacts[i].calculateInternals(duration);
        }
    },

    /**
     * Resolves the velocity issues with the given array of constraints,
     * using the given number of iterations.
     */
    adjustVelocities: function(c, numContacts, duration){
        var velocityChange = [],
            rotationChange = [],
            deltaVel;

        // iteratively handle impacts in order of severity.
        this.velocityIterationsUsed = 0;
        while(this.velocityIterationsUsed < this.velocityIterations){
            // Find contact with maximum magnitude of probable velocity change.
            var max = this.velocityEpsilon;
            var index = numContacts;

            for(var i = 0; i< numContacts; i++ ){
                if(c[i].desiredDeltaVelocity > max){
                    max = c[i].desiredDeltaVelocity;
                    index = i;
                }
            }
            if(index === numContacts){
                break;
            }

            // Match the awake state at the contact
            c[index].matchAwakeState();

            // Do the resolution on the contact that come out top.
            c[index].applyVelocityChange(velocityChange, rotationChange);

            // With the change in velocity of the two rigid bodies, the update of
            // contact velocities means that some of the relative closing
            // velocities nee recomputing.

            for(var i = 0; i < numContacts; i++ ){
                // Check each body in the contact
                for(var b = 0; b < 2; b++) if(c[i].body[b]){
                    // Check for a match with each body in the newly
                    // resolved contact.
                    for(var d = 0; d < 2; d++){
                        if(c[i].body[b] === c[index].body[d]){
                            deltaVel = velocityChange[d].add(
                                rotationChange[d].cross(c[i].relativeContactPosition[b])
                            );

                            // The sign of the change is negative if we're dealing
                            // with the second body in a contact.
                            c[i].contactVelocity = c[i].contactVelocity.add(
                                c[i].contactToWorld.transformTranspose(deltaVel).multiplyScalar(b?-1:1)
                            );
                            c[i].calculateDesiredDeltaVelocity(duration);
                        }
                    }
                }
            }
            this.velocityIterationsUsed += 1;
        }
    },

    /**
     * Resolves the positional issues with the given array of constraints
     * using the given number of iterations.
     */
    adjustPosition: function(c, numContacts, duration){
        var i, index,
            linearChange = [],
            angularChange = [],
            max,
            deltaPosition;

        // Iteratively resolve interpenetration's in order of severity.
        this.positionIterationsUsed = 0;
        while(this.positionIterationsUsed < this.positionIterations){
            // Find biggest penetration
            max = this.positionEpsilon;
            index = numContacts;
            for(i = 0; i < numContacts; i++){
                if(c[i].penetration > max){
                    max = c[i].penetration;
                    index = i;
                }
            }
            if(index === numContacts){
                break;
            }

            // Match the awake state at the contact
            c[index].matchAwakeState();

            // Resolve the penetration.
            c[index].applyPositionChange(linearChange, angularChange, max);


            // Again the action may have change the penetration of other
            // bodies, so we update contacts.
            for(i = 0; i< numContacts; i++){
                // Check each body in the contact
                for(var b = 0; b < 2; b++) if(c[i].body[b]){
                    // Check a match with each body in the newly
                    // resolved contact.
                    for(var d = 0; d < 2;  d++){
                        if(c[i].body[b] === c[index].body[d]){
                            deltaPosition = linearChange[d].add(
                                angularChange[d].cross(c[i].relativeContactPosition[b])
                            );

                            // The sign of the change is positive if we're
                            // dealing with the second body in a contact and
                            // negative otherwise (because we're
                            // subtracting the resolution)..
                            c[i].penetration +=
                                deltaPosition.dot(c[i].contactNormal) * (b?1:-1);
                        }
                    }

                }
            }
            this.positionIterationsUsed += 1;
        }
    }
};