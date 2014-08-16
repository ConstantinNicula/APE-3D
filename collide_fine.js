/**
 * This file contains the fine grained collision detection system.
 * It is used to return contacts between pairs of primitives.
 *
 * There are two groups of tests in this file. Intersection tests
 * use the fastest separating axis method to check if two
 * objects intersect, and collision tests generate the contacts. The
 * collision tests typically use the intersection tests as an early
 * out.
 */

/**
 * Represents a primitive to detect collisions against.
 */
APE.CollisionPrimitive = function(){
    /**
     * The rigid body that is represented by this primitive.
     */
    this.body = null;

    /**
     * The offset of this primitive from the given rigid body.
     */
    this.offset = new APE.Matrix4();

    /**
     * The resultant transform of the primitive. This is
     * calculated by combining the offset of the primitive
     * with the transform of the rigid body.
     */
    this.transform = new APE.Matrix4();
};

APE.CollisionPrimitive.prototype = {
    constructor: APE.CollisionPrimitive,

    /**
     * Calculates the internals for the primitive.
     */
    calculateInternals: function(){
        this.transform = this.body.getTransform().multiply(this.offset);
    },

    /**
     * This is a convenience function to allow access to the
     * axis vectors in the transform for this primitive.
     */
    getAxis: function(index){
        return this.transform.getAxisVector(index);
    },

    /**
     * Returns the resultant transform of the primitive, calculated from
     * the combined offset of the primitive and th transform
     *(orientation and position) of the rigid body to which it is
     * attached.
     */
    getTransform: function(){
        return this.transform.clone();
    }
};

/**
 * Represents a rigid body that can be treated as a sphere
 * for collision detection.
 */
APE.CollisionSphere = function(){
    APE.CollisionPrimitive.call(this);

    /**
     * The radius of this sphere.
     */
    this.radius = 0;
};
APE.extend(APE.CollisionSphere, APE.CollisionPrimitive);

/**
 * The plane is not a primitive: it doesn't represent another
 * rigid body. It is used for contacts with the immovable
 * world geometry.
 */
APE.CollisionPlane = function(){
    /**
     * The plane normal.
     */
    this.direction = new APE.Vector3();

    /**
     * The distance of the plane from the origin.
     */
    this.offset = 0;
};

/**
 * Represents a rigid body that can be treated as an aligned bounding
 * box for collision detection.
 */
APE.CollisionBox = function(){
    APE.CollisionPrimitive.call(this);
    /**
     * Holds the half-sizes of the box along watch of its local axes.
     */
    this.halfSize =  new APE.Vector3();
};
APE.extend(APE.CollisionBox, APE.CollisionPrimitive);

/**
 * A wrapper class that holds fast intersection tests. These
 * can be used to drive the coarse collision detection systems
 * or as an early out in the full collision test below.
 */
APE.IntersectionTests = {
    sphereAndHalfSpace : function(sphere, plane){
        // Find the distance from the origin
        var ballDistance = plane.direction.dot(sphere.getAxis(3)) - sphere.radius;

        // Check for the intersection
        return ballDistance <= plane.offset;
    },
    spherseAndSphere : function(one, two){
        // Find the vector between the objects
        var midline = one.getAxis(3).sub(two.getAxis(3));

        // See if it is large enough
        return midline.squareMagnitude() <
            (one.radius + two.radius) * (one.radius + two.radius);
    },
    boxAndBox : function(){

    },

    /**
     * Does an intersection test on an arbitrarily aligned box and
     * a half-space.
     *
     * The box is given as a transform matrix, including
     * position, and a vector of half-sizes for the extend of the
     * box along each local ais.
     *
     * The half-space is given as a direction (unit) vector and the offset
     * of the limiting plane from the origin, along the
     * given direction.
     */
    boxAndHalfSpace : function(box, plane){
        // Work out the projected radius of the box onto the plane direction
        var projectedRadius = this.transformToAxis(box, plane.direction);

        // Work out how far the box is from the origin
        var boxDistance = plane.direction.dot(box.getAxis(3)) - projectedRadius;

        // Check for the intersection
        return boxDistance <= plane.offset;
    },

    // Helper functions
    transformToAxis: function(box, axis){
        return box.halfSize.x * Math.abs(axis.dot(box.getAxis(0))) +
               box.halfSize.y * Math.abs(axis.dot(box.getAxis(1))) +
               box.halfSize.z * Math.abs(axis.dot(box.getAxis(2)));
    }
};


/**
 * A helper structure that contains information for the detector to use
 * in building its contact data.
 */
APE.CollisionData = function(){
    /**
     * Holds the base of the collision data: the first contact in
     * the array. This is used so that the contact index(below)
     * can be incremented each time a contact is detected, while
     * this pointer points to the first contact found.
     */
    this.contactArray = [];

    /**
     * Holds the maximum number of contacts the array can take.
     */
    this.contactsLeft = 0;

    /**
     * Holds the friction value to write into any collisions.
     */
    this.friction = 0;

    /**
     * Holds the restitution value to write into any collisions.
     */
    this.restitution = 0;

    /**
     * Holds the collision tolerance, even non-colliding objects
     * this close should have collisions generated.
     */
    this.tolerance = 0;
};

APE.CollisionData.prototype = {
    constructor: APE.CollisionData,

    /**
     * Checks if there are more contacts available in the contact
     * data.
     */
    hasMoreContacts: function(){
        return this.contactsLeft > 0;
    },

    /**
     * Resets the data so that it has no used contacts recorded.
     */
    reset: function(maxContacts){
        this.contactsLeft = maxContacts;
        this.contactArray =[];
    },

    /**
     * Notifies the data that the given number of contacts have
     * been added.
     */
    addContacts: function(count){
        // Reduce the number of contacts remaining, add number used
        this.contactsLeft -= count;
    }
};

/**
 * A wrapper class that holds the fine grained collision
 * detection routines.
 *
 * Each of the functions has the same format: it takes the
 * details of two objects, and a pointer to a contacts array to fill.
 * It returns the number of contacts it wrote into the array.
 */
APE.CollisionDetector = {
    sphereAndHalfSpace : function(sphere, plane, data){
        // Make sure we have contacts
        if(data.contactsLeft <= 0){
            return 0;
        }

        // Cache the sphere position
        var position = sphere.getAxis(3);

        // Find the distance from the plane
        var ballDistance = plane.direction.dot(position) -
            sphere.radius- plane.offset;

        if(ballDistance >= 0){
            return 0;
        }

        // Create the contact it has a normal in the plane direction.
        var contact = new APE.Contact();
        contact.contactNormal = plane.direction;
        contact.penetration = -ballDistance;
        contact.contactPoint = position.sub(plane.direction.multiplyScalar(ballDistance + sphere.radius));
        contact.setBodyData(sphere.body, null, data.friction, data.restitution);

        data.contactArray.push(contact);
        data.addContacts(1);

        return 1;
    },

    sphereAndTruePlane : function(sphere, plane , data){
        // Cache the sphere position
        var position = sphere.getAxis(3);

        // Find the distance from the plane
        var centreDistance = plane.direction.dot(position) - plane.offset;

        // Check if we are within radius
        if(centreDistance*centreDistance > sphere.radius*sphere.radius){
           return 0;
        }

        // Check which side of the plane we're on
        var normal = plane.direction.clone();
        var penetration = - centreDistance;
        if(centreDistance < 0){
            normal = normal.multiplyScalar(-1);
            penetration = -penetration;
        }
        penetration += sphere.radius;

        // Create the contact it has a normal in the plane direction.
        var contact = new APE.Contact();
        contact.contactNormal = normal;
        contact.penetration = penetration;
        contact.contactPoint = position.sub(plane.direction.multiplyScalar(centreDistance));
        contact.setBodyData(sphere.body, null, data.friction, data.restitution);

        data.contactArray.push(contact);
        data.addContacts(1);

        return 1;
    },

    sphereAndSphere : function(one, two, data){
        // Make sure we have contacts
        if(data.contactsLeft <= 0){
            return 0;
        }

        // Cache the sphere positions
        var positionOne = one.getAxis(3),
            positionTwo = two.getAxis(3);


        //Find the vector between the objects
        var midline = positionOne.sub(positionTwo);
        var size = midline.magnitude();

        // See if it is large enough.
        if(size <= 0 || size >= one.radius + two.radius){
            return 0;
        }

        // We manually create the normal, because we have the
        // size at hand.
        var normal = midline.multiplyScalar(1/size);
        var contact = new APE.Contact();
        contact.contactNormal = normal;
        contact.contactPoint = positionOne.add(positionTwo).multiplyScalar(0.5);
        contact.penetration = (one.radius + two.radius - size);
        contact.setBodyData(one.body, two.body, data.friction, data.restitution);

        data.contactArray.push(contact);
        data.addContacts(1);

        return 1;
    },

    /**
     * Does a collision test on a collision box and a plane representing a
     * half-space (this normal of the plane points out of the half-space).
     */
    boxAndHalfSpace : function(box, plane, data){
        // Make sure we have contacts
        if(data.contactsLeft <= 0){
            return 0;
        }

        // Check for intersection
        if(!APE.IntersectionTests.boxAndHalfSpace(box, plane)){
            return 0;
            console.log('ok');
        }

        // We have an intersection, so find the intersection points. We can make
        // do with only checking vertices. If the box is resting on a plane or on an
        // edge, it will be reported as four or two contact points.

        // Go through each combination of + and - for each half-size
        var mults = [[1, 1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, 1],
                     [1, 1, -1], [-1, 1, -1], [1, -1, -1], [-1, -1, -1]];

        var contactsUsed = 0;
        var contact;
        for(var i = 0; i < 8; i++){
            // Calculate the position of each vertex
            var vertexPos = new APE.Vector3(mults[i][0], mults[i][1], mults[i][2]);
            vertexPos = vertexPos.componentProduct(box.halfSize);
            vertexPos = box.transform.transform(vertexPos);

            // Calculate the distance from the plane.
            var vertexDistance = vertexPos.dot(plane.direction);

            // Compare this to the plane's distance
            if(vertexDistance <= plane.offset){
                // Create the contact data.
                // The contact point is halfway between the vertex and the
                // plane - we multiply the direction by half the separation
                // distance and add the vertex location.
                contact = new APE.Contact();
                contact.contactPoint = plane.direction.multiplyScalar((plane.offset - vertexDistance)/2);
                contact.contactPoint = contact.contactPoint.add(vertexPos);
                contact.contactNormal = plane.direction;
                contact.penetration = plane.offset - vertexDistance;

                // Write the appropriate data
                contact.setBodyData(box.body, null, data.friction, data.restitution);

                // Move onto the next contact
                data.contactArray.push(contact);
                contactsUsed += 1;

                if(contactsUsed === data.contactsLeft){
                    return contactsUsed;
                }
            }

        }
        data.addContacts(contactsUsed);
        return contactsUsed;
    },

    boxAndBox : function(one, two, data){
        // Find the vector between the two centres
        var toCentre = two.getAxis(3).sub(one.getAxis(3));
        // We start assuming there is no contact
        var res = {
            pen : Number.MAX_VALUE,
            best : 0xffffff
        };

        // Now we check each axes, returning if it gives us
        // a separating axis, and keeping track of the axis with the
        // smallest penetration otherwise.
        if(!this.tryAxis(one, two, one.getAxis(0), toCentre, 0, res)){
            return 0;
        }
        if(!this.tryAxis(one, two, one.getAxis(1), toCentre, 1, res)){
            return 0;
        }
        if(!this.tryAxis(one, two, one.getAxis(2), toCentre, 2, res)){
            return 0;
        }

        if(!this.tryAxis(one, two, two.getAxis(0), toCentre, 3, res)){
            return 0;
        }
        if(!this.tryAxis(one, two, two.getAxis(1), toCentre, 4, res)){
            return 0;
        }
        if(!this.tryAxis(one, two, two.getAxis(2), toCentre, 5, res)){
            return 0;
        }

        // Store the best axis-major, in case we run into almost
        // parallel edge collisions later.
        var bestSingleAxis = res.best;

        if(!this.tryAxis(one, two,one.getAxis(0).cross(two.getAxis(0)) , toCentre, 6, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(0).cross(two.getAxis(1)) , toCentre, 7, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(0).cross(two.getAxis(2)) , toCentre, 8, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(1).cross(two.getAxis(0)) , toCentre, 9, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(1).cross(two.getAxis(1)) , toCentre, 10, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(1).cross(two.getAxis(2)) , toCentre, 11, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(2).cross(two.getAxis(0)) , toCentre, 12, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(2).cross(two.getAxis(1)) , toCentre, 13, res)){
            return 0;
        }
        if(!this.tryAxis(one, two,one.getAxis(2).cross(two.getAxis(2)) , toCentre, 14, res)){
            return 0;
        }


        // We now know there's a collision, and we know which
        // of the case gave the smallest penetration. We now
        // can deal with it in different ways depending on
        // the case.
        if(res.best < 3){
            // We've got a vertex of box two on a face of box one.
            this.fillPointFaceBoxBox(one, two, toCentre, data, res.best, res.pen);
            data.addContacts(1);
            return 1;
        }else if(res.best < 6){
            // We've got a vertex of box one on a face of box two.
            // We use the same algorithm as above, but swap around
            // one and two (and therefore also the vector between their
            // centres).
            this.fillPointFaceBoxBox(two, one, toCentre.multiplyScalar(-1), data, res.best-3, res.pen);
            data.addContacts(1);
            return 1;
        }else{
            // We've got an edge-edge contact. Find out which axes
            res.best -= 6;
            var oneAxisIndex = Math.floor(res.best / 3);
            var twoAxisIndex = res.best % 3;
            var oneAxis = one.getAxis(oneAxisIndex);
            var twoAxis = two.getAxis(twoAxisIndex);
            var axis = oneAxis.cross(twoAxis);
            axis.normalize();
            // The axis should point from box one to box two.
            if(axis.dot(toCentre)> 0){
                axis = axis.multiplyScalar(-1);
            }

            // We have the axes, but not the edges each axis has 4 edges parallel
            // to it, we need to find out which of the 4 for each object. We do that
            // by finding the point in the centre of the edge. We know its component
            // in the direction of the box's collision axis is zero (its a mid-point)
            // and we determine which of the extremes in each of the other axes is closest.
            var ptOnOneEdge = one.halfSize.clone();
            var ptOnTwoEdge = two.halfSize.clone();
            for(var i = 0; i < 3; i++){
                if(i === oneAxisIndex){
                    if(i === 0) {
                        ptOnOneEdge.x = 0;
                    }else if(i===1){
                        ptOnOneEdge.y = 0;
                    }else{
                        ptOnOneEdge.z = 0
                    }
                }else if(one.getAxis(i).dot(axis) > 0){
                    if(i === 0) {
                        ptOnOneEdge.x = -ptOnOneEdge.x;
                    }else if(i===1){
                        ptOnOneEdge.y = -ptOnOneEdge.y;
                    }else{
                        ptOnOneEdge.z = -ptOnOneEdge.z;
                    }
                }
                if(i === twoAxisIndex){
                    if(i === 0) {
                        ptOnTwoEdge.x = 0;
                    }else if(i===1){
                        ptOnTwoEdge.y = 0;
                    }else{
                        ptOnTwoEdge.z = 0
                    }
                }else if(two.getAxis(i).dot(axis) < 0){
                    if(i === 0) {
                        ptOnTwoEdge.x = -ptOnTwoEdge.x;
                    }else if(i===1){
                        ptOnTwoEdge.y = -ptOnTwoEdge.y;
                    }else{
                        ptOnTwoEdge.z = -ptOnTwoEdge.z;
                    }
                }
            }

            // Move them into world coordinates (they are already oriented
            // correctly, since they have been derived from the axes).
            ptOnOneEdge = one.transform.transform(ptOnOneEdge);
            ptOnTwoEdge = two.transform.transform(ptOnTwoEdge);

            // So we have a point and a direction for the colliding edges.
            // We need to fnd out the point closest approach of the two
            // line-segments.
            var vertex = this.contactPoint(
                ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
                ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
                bestSingleAxis > 2
            );

            // We can fill the contact.
            var contact = new APE.Contact();
            contact.penetration = res.pen;
            contact.contactNormal = axis;
            contact.contactPoint = vertex;
            contact.setBodyData(one.body, two.body,
                data.friction, data.restitution);
            data.contactArray.push(contact);
            data.addContacts(1);
            return 1;
        }
    },

    boxAndPoint : function(){

    },

    boxAndSphere : function(box, sphere, data){
        // Transform the centre of the sphere into box coordinates
        var centre = sphere.getAxis(3);
        var relCentre = box.transform.transformInverse(centre);

        // Early out check to see if we can exclude the contact
        if(Math.abs(relCentre.x) - sphere.radius > box.halfSize.x||
           Math.abs(relCentre.y) - sphere.radius > box.halfSize.y||
           Math.abs(relCentre.z) - sphere.radius > box.halfSize.z){
            return 0;
        }

        var closestPt = new APE.Vector3();
        var dist;
        // Clamp each coordinate to the box.
        dist = relCentre.x;
        if(dist > box.halfSize.x){
            dist = box.halfSize.x;
        }
        if(dist < -box.halfSize.x){
            dist = - box.halfSize.x;
        }
        closestPt.x = dist;

        dist = relCentre.y;
        if(dist > box.halfSize.y){
            dist = box.halfSize.y;
        }
        if(dist < -box.halfSize.y){
            dist = -box.halfSize.y;
        }
        closestPt.y = dist;

        dist = relCentre.z;
        if(dist > box.halfSize.z){
            dist = box.halfSize.z;
        }
        if(dist < -box.halfSize.z){
            dist = -box.halfSize.z;
        }
        closestPt.z = dist;
        // Check we're in contact
        dist = closestPt.sub(relCentre).squareMagnitude();
        if(dist > sphere.radius * sphere.radius){
            return 0;
        }

        // Write the contact
        var closestPtWorld = box.transform.transform(closestPt);

        var contact = new APE.Contact();
        contact.contactNormal = closestPtWorld.sub(centre);
        contact.contactNormal.normalize();
        contact.contactPoint = closestPtWorld;
        contact.penetration = sphere.radius - Math.sqrt(dist);
        contact.setBodyData(box.body, sphere.body, data.friction, data.restitution);

        data.contactArray.push(contact);
        data.addContacts(1);
        return 1;
    },

    //Helper Functions
    transformToAxis: function(box, axis){
        return box.halfSize.x * Math.abs(axis.dot(box.getAxis(0)) ) +
               box.halfSize.y * Math.abs(axis.dot(box.getAxis(1)) ) +
               box.halfSize.z * Math.abs(axis.dot(box.getAxis(2)) );
    },

    /**
     * This function checks if the two boxes overlap
     * along the given axis. The final parameter toCentre
     * is used to pass in the vector between the boxes centre
     * points, to avoid having to recalculate it each time.
     */
    penetrationOnAxis: function(one, two , axis, toCentre){
        // Project the half-size of one onto axis
        var oneProject = this.transformToAxis(one, axis);
        var twoProject = this.transformToAxis(two, axis);

        // Project this onto the axis
        var distance = Math.abs(toCentre.dot(axis));

        // Check for overlap
        return oneProject + twoProject - distance;
    },

    tryAxis: function(one, two, axis, toCentre, index, res){
        // Make sure we have a normalized axis and don't check almost parallel axes
        if(axis.squareMagnitude() < 0.0001){
            return true;
        }
        axis.normalize();

        var penetration = this.penetrationOnAxis(one, two, axis, toCentre);
        if(penetration < 0){
            return false;
        }
        if(penetration < res.pen){
            res.pen = penetration;
            res.best = index;
        }
        return true;
    },

    fillPointFaceBoxBox : function(one, two, toCentre, data, best, pen){
        // This method is called when we know that a vertex from
        // box two is in contact with box one.

        var contact = new APE.Contact();
        // We know which of the six the collision is on (best),
        // But we need to work out which of the two faces  on
        // this axis.

        var normal = one.getAxis(best);
        if(one.getAxis(best).dot(toCentre) > 0){
            normal = normal.multiplyScalar(-1);
        }

        // Work out which vertex of the box we're colliding with.
        // Using toCentre doesn't work.

        var vertex = two.halfSize.clone();
        if(two.getAxis(0).dot(normal)<0){
            vertex.x = -vertex.x;
        }
        if(two.getAxis(1).dot(normal)<0){
            vertex.y = -vertex.y;
        }
        if(two.getAxis(2).dot(normal)<0){
            vertex.z = -vertex.z;
        }

        // Create the contact data
        contact.contactNormal = normal;
        contact.penetration = pen;
        contact.contactPoint = two.getTransform().transform(vertex);
        contact.setBodyData(one.body, two.body,
            data.friction, data.restitution);
        data.contactArray.push(contact);
    },

    contactPoint: function(pOne, dOne, oneSize, pTwo, dTwo, twoSize, useOne){

        var smOne = dOne.squareMagnitude();
        var smTwo = dTwo.squareMagnitude();
        var dpOneTwo = dTwo.dot(dOne);

        var toSt = pOne.sub(pTwo);
        var dpStaOne = dOne.dot(toSt);
        var dpStaTwo = dTwo.dot(toSt);

        var denom = smOne * smTwo - dpOneTwo * dpOneTwo;
        //Zero denominator indicates parrallel lines
        if(Math.abs(denom) < 0.0001){
            return useOne? pOne: pTwo;
        }

        var mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) /denom;
        var mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne)/ denom;

        // If either of the edges has the nearest point out
        // of bounds, then the edges aren't crossed, we have
        // an edge-face contact. Our point is on the edge, which
        // we know from the useOne parameter.
        if(mua > oneSize||
            mua < -oneSize ||
            mub > twoSize||
            mub < -twoSize)
            {
            return useOne? pOne: pTwo;
        }else
        {
            var cOne = pOne.add(dOne.multiplyScalar(mua));
            var cTwo = pTwo.add(dTwo.multiplyScalar(mub));
            return cOne.multiplyScalar(0.5).add(cTwo.multiplyScalar(0.5));
        }

    }

};