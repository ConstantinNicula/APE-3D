/**
 * Joints link together two rigid bodies and make sure they don not
 * separate. In a general physics engine there may be many
 * different type of joints, that reduce the number of relative
 * degrees of freedom between two objects. This joint is a common
 * position joint: each object has a location(given in body-coordinates)
 * that will be kept at the same point in the simulation.
 */

APE.Joint = function(){
    /**
     * Holds the two rigid bodies that are connected by this joint.
     */
    this.body = [];

    /**
     * Holds the relative location of the connection for each body
     * given in local coordinates.
     */
    this.position = [];

    /**
     * Holds the maximum displacement at the joint before the
     * joint is considered to be violated. This is normally a small,
     * epsilon value. It can be larger, however, in which case
     * the joint will behave if an inelastic cable joined the
     * bodies at their locations.
     */
    this.error = 0;
};

APE.Joint.prototype = {
    constructor: APE.Joint,

    /**
     * Configures the joint in one go.
     */
    set: function(a, a_pos, b, b_pos, error){
        this.body[0] = a;
        this.body[1] = b;

        this.position[0] = a_pos;
        this.position[1] = b_pos;

        this.error = error;
    },

    /**
     * Generates the contacts required to restore the joint if it
     * has been violated.
     */
    addContact: function(contacts, limit){
        // Calculate the position of each connection point in world coordinates.
        var a_pos_world = this.body[0].getPointInWorldSpace(this.position[0]);
        var b_pos_world = this.body[1].getPointInWorldSpace(this.position[1]);

        // Calculate the length of the joint
        var a_to_b = b_pos_world.sub(a_pos_world);
        var normal = a_to_b.clone();
        normal.normalize();
        var length = a_to_b.magnitude();

        // Check if it is violated
        if(Math.abs(length) > this.error){
            var contact = new APE.Contact();
            contact.body[0] = this.body[0];
            contact.body[1] = this.body[1];
            contact.contactNormal = normal;
            contact.contactPoint = a_pos_world.add(b_pos_world).multiplyScalar(0.5);
            contact.penetration = length-this.error;
            contact.friction = 1;
            contact.restitution = 0;
            contacts.push(contact);
            return 1;
        }
        return 0;
    }
};