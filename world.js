/**
 * This file contains the definitions for a structure to hold any number
 * of rigid bodies, and to manage their simulation.
 */

/**
 * Holds a single rigid body in a linked list of bodies.
 */
APE.BodyRegistration = function(){
    this.body = null;
    this.next = null;
};

/**
 * Holds one contact generator in a linked list.
 */
APE.ContactGenRegistration = function(){
    this.gen = null;
    this.next = null;
};

/**
 * The world represents an independent simulation of physics. It
 * keeps track of a set of rigid bodies, and provides the means to
 * update them all.
 */

/**
 * Creates a new simulator that can handle up to the given
 * number of contacts per frame. You can also optionally give
 * a number of contact- resolution iterations to use. If you
 * don't give a number of iterations, then four times the
 * number of detected contacts will be used for each frame.
 */
APE.World = function(maxContacts, iterations){
   /**
     * True if the world should calculate the number of iterations
     * to give the contact resolver at each frame.
     */
    this.calculateIterations = iterations === undefined;

    /**
     * Holds the head of the list of registered bodies.
     */
    this.firstBody = null;

    /**
     * Holds the resolver for sets of contacts.
     */
    this.resolver = null;

    /**
     * Holds the head of the list of contact generators.
     */
    this.firstContactGen = null;

    /**
     * Holds an array of contact, for filling by the contact
     * generators.
     */
    this.contacts = [];

    /**
     * Holds the maximum number of contacts allowed (the size of the
     * contacts array).
     */
    this.maxContacts = maxContacts;
};

APE.World.prototype = {
    constructor: APE.World,

    /**
     * Calls each of the registered contact generators to report
     * their contacts. Returns the number of generated contacts.
     */
    generateContacts: function(){
        var limit = this.maxContacts;
        var nextContact = 0;

        var reg = this.firstContactGen;
        while(reg !== null){
            var used = reg.gen.addContact(this.contacts[nextContact], limit);
            limit -= used;
            nextContact += used;

            // We've run ot of contacts to fill. This means we're missing
            // contacts.
            if(limit <= 0){
                break;
            }

            reg = reg.next;
        }

        // Return the number of contacts used.
        return this.maxContacts - limit;
    },

    /**
     * Processes all the physics for the world.
     */
    runPhysics: function(duration){
        // First apply the force generators
        // registry.updateForces(duration);

        // Then integrate the objects
        var reg = this.firstBody;
        while(reg !== null){
            // Remove all forces from the accumulator.
            reg.body.integrate(duration);

            // Get the next registration
            reg = reg.next;
        }

        // Generate contacts
        var usedContacts = this.generateContacts();

        // And process them
        if(this.calculateIterations){
            this.resolver.setIterations(usedContacts * 4);
        }
        this.resolver.resolveContacts(this.contacts, usedContacts, duration);
    },

    /**
     * Initialises the world for a simulation frame. This clears
     * the force and torque accumulators for bodies in the
     * world. After calling this, the bodies can have their forces
     * and torques for this frame added.
     */
    startFrame: function(){
        var reg = this.firstBody;
        while(reg !== null){
            // Remove all forces from the accumulator
            reg.body.clearAccumulators();
            reg.body.calculateDerivedData();

            // Get the next registration.
            reg = reg.next;
        }
    }
};