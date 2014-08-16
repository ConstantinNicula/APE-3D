/**
 * The main demo class definition.
 */
SailBoatDemo = function(){
    this.windspeed = new APE.Vector3(0, 0, 0);
    this.buoyancy = new APE.Buoyancy(
        new APE.Vector3(0, 0.5, 0),
        1.0,
        3.0,
        1.6
    );
    this.sail = new APE.Aero(
        new APE.Matrix3(-1,0,0,  0,0,0,  0,0,-1),
        new APE.Vector3(0, 2.0, 0),
        this.windspeed
    );
    this.sailboat = new APE.RigidBody();
    this.registry = new APE.ForceRegistry();

    //this.r;

    // Set up the boat's rigid body.
    this.sailboat.setPosition(new APE.Vector3(0, 1.6, 0));
    this.sailboat.setOrientation(new APE.Quaternion(1, 0, 0, 0));

    this.sailboat.setVelocity(new APE.Vector3(0, 0, 0));
    this.sailboat.setRotation(new APE.Vector3(0, 0, 0));

    this.sailboat.setMass(200.0);
    var it = new APE.Matrix3();
    it.setBlockInertiaTensor(new APE.Vector3(2, 1, 1), 100);
    this.sailboat.setInertiaTensor(it);

    this.sailboat.setDamping(0.8, 0.8);

    this.sailboat.setAcceleration(APE.GRAVITY);
    this.sailboat.calculateDerivedData();

    this.sailboat.setAwake(true);
    this.sailboat.setCanSleep(false);

    this.registry.add(this.sailboat, this.sail);
    this.registry.add(this.sailboat, this.buoyancy);
};

SailBoatDemo.prototype ={
    constructor: SailBoatDemo,

    update: function(){
        // Find the duration of the last frame in seconds
        var duration = 1/60;

        // Start with no force or acceleration.
        this.sailboat.clearAccumulators();

        // Add the forces acting on the boat.
        this.registry.updateForces(duration);

        // Update the boat's physics.
        this.sailboat.integrate(duration);

        // Change the wind speed.
        this.windspeed.x = this.windspeed.x * 0.9 + (Math.random() -Math.random())*10;
        this.windspeed.y = this.windspeed.y * 0.9;
        this.windspeed.z = this.windspeed.z * 0.9 + (Math.random() -Math.random())*10;

    },

    createBoat: function(){
        var sailboat = new THREE.Object3D();
        var material = new THREE.MeshLambertMaterial({color: 0xf0f0f0});

        // left Hull
        var lhull = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.4, 0.4), material);
        lhull.position.set(0, 0, -1.0);
        sailboat.add(lhull);

        // right hull
        var rhull = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.4, 0.4), material);
        rhull.position.set(0, 0, 1.0);
        sailboat.add(rhull);

        // deck
        var deck = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.1, 2.0), material);
        deck.position.set(0, 0.3, 0);
        sailboat.add(deck);

        // Mast
        var mast = new THREE.Mesh(new THREE.BoxGeometry(0.1, 3, 0.1), material);
        mast.position.set(0, 1.8, 0);
        sailboat.add(mast);

        return sailboat;
    }
};