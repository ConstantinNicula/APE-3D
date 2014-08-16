/**
 * This file contains the coarse collision detection system. It is
 * used to return pairs of objects that may be in contact, which can
 * then be tested using fine grained methods.
 */

/**
 * Represents a bounding sphere that can be tested for overlap.
 * Creates a new bounding sphere at the given centre and radius.
 */
APE.BoundingSphere = function(centre, radius){
    this.centre = centre;
    this.radius = radius;
};

APE.BoundingSphere.prototype = {
    constructor: APE.BoundingSphere,
    /**
     * Creates a bounding sphere to enclose the two given bounding
     * spheres.
     */
    enclose: function(one, two){
        var centreOffset = two.centre.sub(one.centre),
            distance = centreOffset.squareMagnitude(),
            radiusDiff = two.radius - one.radius;

        // Check if the larger sphere encloses the small one
        if(radiusDiff*radiusDiff >= distance){
            if(one.radius > two.radius){
                this.centre = one.centre.clone();
                this.radius = one.radius;
            }else{
                this.centre = two.centre.clone();
                this.radius = two.radius;
            }
        }else{
            // Otherwise we need to work with partially
            // overlapping spheres.
            distance = Math.sqrt(distance);
            this.radius = (distance + one.radius + two.radius) * 0.5;

            // The new centre is based on one's centre, moved towards
            // two's centre by amount proportional to the spheres'
            // radii.
            this.centre = one.centre.clone();
            if(distance > 0){
                this.centre = this.centre.add(centreOffset.multiplyScalar((this.radius - one.radius)/ distance));
            }
        }
    },

    /**
     * Checks if the bounding sphere overlaps with the other given
     * bounding sphere.
     */
    overlaps : function(other){
        var distanceSquared = this.centre.sub(other.centre).squareMagnitude();
        return distanceSquared <(this.radius + other.radius)*(this.radius + other.radius);
    },

    /**
     * Reports how much this bounding sphere would have to grow
     * by to incorporate te given bounding sphere. Note that this
     * calculation returns a value not in any particular units
     * (its not a volume growth). In fact the best implementation
     * takes into account the growth in surface area(after the
     * Goldsmith-Salmon algorithm for tree construction).
     */
    getGrowth: function(other){
        var newSphere = new APE.BoundingSphere();
        newSphere.enclose(this, other);

        // We return a value proportional to the change in surface
        // area of the sphere.
        return newSphere.radius * newSphere.radius - this.radius*this.radius;
    },

    /**
     * Return the volume of this bounding volume. This is used
     * to calculate how to recurs into the bounding volume tree.
     * For a bounding sphere it is a simple calculation.
     */
    getSize: function(){
        return 1.333333 * Math.PI * this.radius * this.radius * this.radius;
    }
};

/**
 * Stores a potential contact to check later.
 */
APE.PotentialContact = function(){
    /**
     * Holds the bodies that might be in contact.
     */
    this.body = [];
};


/**
 * Creates a new node in the hierarchy with the given parameters.
 * This class uses a binary tree to store the bounding volumes.
 */

APE.BVHNode = function(parent, volume, body){
    /**
     * Holds the child nodes of this node.
     */
    this.children = [];

    /**
     * Holds a single bounding volume encompassing all the
     * descendants of this node.
     */
    this.volume = volume;

    /**
     * Holds the rigid body at this node of the hierarchy.
     * Only leaf nodes can have a rigid body defined (see isLeaf).
     * Note that it is possible to rewrite the algorithm in this class
     * to handle objects at all levels of the hierarchy, but the code
     * provided ignores this vector unless the first child is NULL.
     */
    this.body = (body !== undefined)? body : null;

    /**
     * Holds the node immediately above us in the tree.
     */
    this.parent = parent;
};

APE.BVHNode.prototype = {
    constructor: APE.BVHNode,

    /**
     * Checks if this node is at the bottom of the hierarchy.
     */
    isLeaf: function(){
        return (this.body !== null);
    },

    /**
     * Checks the potential contacts from this node downwards in
     * the hierarchy, writing them to the given array (up to the given
     * limit). Returns the number of potential contacts it found.
     */
    getPotentialContacts: function(contacts, limit){
        // Early out if we don't have the room for contacts, or
        // if we're a leaf node.
        if(this.isLeaf()|| limit === 0 ){
            return;
        }

        // Get the potential contacts of one of our children with
        // the other.
        return this.children[0].getPotentialContactsWith(this.children[1],
            contacts, limit);
    },

    /**
     * Inserts the given rigid body, with the given bounding volume,
     * into the hierarchy. This may involve the creation of further
     * volume nodes.
     */
    insert: function(newBody, newVolume){
        // If we are a leaf, then the only option is to spawn two
        // new children and place the new body in one.
        if(this.isLeaf()){
            //Child one is a copy of us.
            this.children[0] = new APE.BVHNode(this, this.volume, this.body);

            // Child two holds the new body
            this.children[1] = new APE.BVHNode(this, newVolume, newBody);

            // And we now loose the body(we're no longer a leaf)
            this.body = null;

            // We now recalculate our bounding volume
            this.recalculateBoundingVolume();
        }else{
            // Otherwise we need to work out which child gets to keep
            // the inserted body. We give it to whoever would grow
            // the least to incorporate it.

            if(this.children[0].volume.getGrowth(newVolume)<
               this.children[1].volume.getGrowth(newVolume)){
                this.children[0].insert(newBody, newVolume);
            }else{
                this.children[1].insert(newBody, newVolume);
            }

        }
    },

    /**
     * Deletes this node, removing it first from the hierarchy, along
     * with its associated rigid body nodes. This method deletes the node
     * and all its children(but obviously not the rigid bodies). This
     * also has the effect of deleting the sibling of this node, and
     * changing the parent node so that it contains the data currently
     * in that sibling. Finally it forces the hierarchy above the
     * current node to reconsider its bounding volume.
     */
    delete: function(){
        // If we don't have a parent, the we ignore the sibling
        // processing
        if(this.parent !== null){
            // Find our sibling
            var sibling;
            if(this.parent.children[0] === this){
                sibling = this.parent.children[1];
            }else{
                sibling = this.parent.children[0];
            }

            // Write its data to iur parent
            this.parent.volume = sibling.volume;
            this.parent.body = sibling.body;
            this.parent.children = sibling.children;

            // Delete the sibling ( we blank its parent and
            // children to avoid processing/ deleting them)
            sibling.parent = null;
            sibling.body = null;
            sibling.children = [];
            sibling.delete();


            // Recalculate the parent's bounding volume.
            this.parent.recalculateBoundingVolume();
        }

        // Delete our children (again we remove their
        // parent data so we don't try to process their sibling as
        // they are deleted).
        if(this.children[0]){
            this.children[0].parent = null;
            this.children[0].delete();
        }
        if(this.children[1]){
            this.children[1].parent = null;
            this.children[1].delete();
        }
    },

    /**
     * Checks for overlapping between nodes in the hierarchy. Note that
     * any bounding volume should have an overlaps method implemented
     * that checks for overlapping with another object of its own type.
     */
    overlaps: function(other){
        return this.volume.overlaps(other.volume);
    },

    /**
     * Checks the potential contacts between this node and the given
     * other node, writing them to the given array(up to the given limit).
     * Returns the number of potential contacts it found.
     */
    getPotentialContactsWith: function(other,contacts, limit){
        // Early out if we don't overlap or if we have no room
        // to report contacts
        if(!this.overlaps(other) || limit === 0){
            return 0;
        }

        // If we're both at a leaf node, then we have a potential contact
        if(this.isLeaf() && other.isLeaf()){
            var contact = new APE.PotentialContact();
            contact.body[0] = this.body;
            contact.body[1] = other.body;
            contacts.push(contact);
            return 1;
        }

        // Determine which node to descend into. If either is
        // a leaf, then descend the other. If both are branches,
        // the use the one with the largest size.
        if(other.isLeaf() ||
            (!this.isLeaf() && this.volume.getSize() >= other.volume.getSize())){
            // Recurs into this node.
            var count = this.children[0].getPotentialContactsWith(other, contacts, limit);

            // Check we have enough slots to do the other side too
            if(limit > count) {
                return count + this.children[1].getPotentialContactsWith(other,contacts, limit - count);
            }else{
                return count;
            }
        }else{
            // Recurs into the other node.
            var count = this.getPotentialContactsWith(other.children[0], contacts, limit);

            // Check we have enough slots to do the other side too
            if(limit > count){
                return count + this.getPotentialContactsWith(other.children[1],contacts, limit - count)
            }else{
                return count;
            }
        }
    },

    /**
     * For non-leaf nodes, this method recalculates the bounding volume
     * based on the bounding volume of its children.
     */
    recalculateBoundingVolume: function(){
        if(this.isLeaf()){
            return;
        }

        this.volume = new APE.BoundingSphere();
        this.volume.enclose(this.children[0].volume, this.children[1].volume);

        // Recurs up the tree.
        if(this.parent !== null){
            this.parent.recalculateBoundingVolume();
        }
    }
};
