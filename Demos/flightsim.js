/**
 * The main class definition.
 */
function FlightSim(){

    this.windspeed = new APE.Vector3(0, 0, 0);

    this.left_wing = new APE.AeroControl(
        new APE.Matrix3(0, 0, 0,  -1, -0.5, 0,  0, 0, 0),
        new APE.Matrix3(0, 0, 0,  -0.995, -0.5, 0,  0, 0, 0),
        new APE.Matrix3(0, 0, 0,  -1.005, -0.5, 0,  0, 0, 0),
        new APE.Vector3(-1, 0, 2),
        this.windspeed
    );

    this.right_wing = new APE.AeroControl(
        new APE.Matrix3(0, 0, 0,  -1, -0.5, 0,  0, 0, 0),
        new APE.Matrix3(0, 0, 0,  -0.995, -0.5, 0,  0, 0, 0),
        new APE.Matrix3(0, 0, 0,  -1.005, -0.5, 0,  0, 0, 0),
        new APE.Vector3(-1, 0, -2),
        this.windspeed
    );

    this.rudder = new APE.AeroControl(
        new APE.Matrix3(0, 0, 0,  0, 0, 0,  0, 0, 0),
        new APE.Matrix3(0, 0, 0,  0, 0, 0,  0.01, 0, 0),
        new APE.Matrix3(0, 0, 0,  0, 0, 0,  -0.01, 0, 0),
        new APE.Vector3(2, 0.5, 0),
        this.windspeed
    );

    this.tail = new APE.Aero(
        new APE.Matrix3(0, 0, 0,  -1, -0.5, 0,  0, 0, -0.1),
        new APE.Vector3(2.0, 0, 0),
        this.windspeed
    );

    this.left_wing_control = 0;
    this.right_wing_control = 0;
    this.rudder_control = 0;

    this.registry = new APE.ForceRegistry();
    this.aircraft = new APE.RigidBody();

    // Set up the aircraft rigid body.
    this.resetPlane();

    this.aircraft.setMass(2.5);
    var it = new APE.Matrix3();
    it.setBlockInertiaTensor(new APE.Vector3(2, 1, 1), 1);
    this.aircraft.setInertiaTensor(it);

    this.aircraft.setDamping(0.8, 0.8);

    this.aircraft.setAcceleration(new THREE.Vector3(0, -9.81 , 0));
    this.aircraft.calculateDerivedData();

    this.aircraft.setAwake(true);
    this.aircraft.setCanSleep(false);

    this.registry.add(this.aircraft, this.left_wing);
    this.registry.add(this.aircraft, this.right_wing);
    this.registry.add(this.aircraft, this.rudder);
    this.registry.add(this.aircraft, this.tail);

    console.log(this.aircraft);
}

FlightSim.prototype ={
    constructor: FlightSim,

    resetPlane: function(){
        this.aircraft.setPosition(new APE.Vector3(0, 0, 0));
        this.aircraft.setOrientation(new APE.Quaternion(1, 0, 0, 0));

        this.aircraft.setVelocity(new APE.Vector3(0, 0, 0));
        this.aircraft.setRotation(new APE.Vector3(0, 0, 0));

        this.left_wing_control = 0;
        this.right_wing_control = 0;
        this.rudder_control = 0;

        // Update the control surfaces.
        this.left_wing.setControl(this.left_wing_control);
        this.right_wing.setControl(this.right_wing_control);
        this.rudder.setControl(this.rudder_control);

    },

    update: function(){
        // Find the duration of the last frame
        var duration  = 1/60;

        // Start with no forces or acceleration.
        this.aircraft.clearAccumulators();

        // Add the propeller force
        var propulsion = new APE.Vector3(-10, 0, 0);
        propulsion = this.aircraft.getTransform().transformDirection(propulsion);
        this.aircraft.addForce(propulsion);

        // Add the forces acting on the aircraft.
        this.registry.updateForces(duration);
        //this.rudder.updateForce(this.aircraft, duration);
        //console.log(this.aircraft.forceAccum);

        // Update the aircraft's physics.
        this.aircraft.integrate(duration);

        // Do a very basic collision detection and response with the ground.
        var pos = this.aircraft.getPosition();
        if(pos.y < 0){
            pos.y = 0;
            this.aircraft.setPosition(pos);

            if(this.aircraft.getVelocity().y < -10.0){
                this.resetPlane();
            }
        }
    },

    key: function(key){
        switch(key){
            case 'q' : case'Q':
                this.rudder_control += 0.1;
                break;

            case 'e' : case 'E':
                this.rudder_control -= 0.1;
                break;

            case 'w': case 'W':
                this.left_wing_control -= 0.1;
                this.right_wing_control -= 0.1;
                break;

            case 's': case 'S':
                this.left_wing_control += 0.1;
                this.right_wing_control += 0.1;
                break;

            case 'd':case 'D':
                this.left_wing_control -= 0.1;
                this.right_wing_control += 0.1;
                break;

            case 'a':case 'A':
                this.left_wing_control += 0.1;
                this.right_wing_control -= 0.1;
                break;

            case 'x':case'X':
                this.left_wing_control = 0;
                this.right_wing_control = 0;
                this.rudder_control = 0;
                break;

            case 'r':case'R':
                this.resetPlane();
                break;
        }

        // Make sure the controls are in range
        if(this.left_wing_control < -1){
            this.left_wing_control = -1;
        }else if(this.left_wing_control > 1){
            this.left_wing_control = 1;
        }

        if(this.right_wing_control < -1){
            this.right_wing_control = -1;
        }else if(this.right_wing_control > 1){
            this.right_wing_control = 1;
        }

        if(this.rudder_control < -1){
            this.rudder_control = -1;
        }else if(this.rudder_control > 1){
            this.rudder_control = 1;
        }

        // Update the control surfaces.
        this.left_wing.setControl(this.left_wing_control);
        this.right_wing.setControl(this.right_wing_control);
        this.rudder.setControl(this.rudder_control);
    },

    createAircraft : function(){
        var aircraft = new THREE.Object3D();
        var material = new THREE.MeshLambertMaterial({color: 0xf0f0f0});

        // Fuselage
        var fuselage = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.8, 1.0), material);
        fuselage.position.set(-0.5, 0, 0);
        aircraft.add(fuselage);

        // Rear Fuselage
        var rearFuselage = new THREE.Mesh(new THREE.BoxGeometry(2.75, 0.5, 0.5), material);
        rearFuselage.position.set(1, 0.15, 0);
        aircraft.add(rearFuselage);

        // Wings
        var wings = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.1, 6.0), material);
        wings.position.set(0, 0.3, 0);
        aircraft.add(wings);

        // Rudder
        var rudder = new THREE.Mesh(new THREE.BoxGeometry(0.75, 1.15, 0.1), material);
        rudder.position.set(2.0, 0.775, 0);
        aircraft.add(rudder);

        // Tail-plane
        var tail =new THREE.Mesh( new THREE.BoxGeometry(0.85, 0.1, 2.0), material);
        tail.position.set(1.9, 0, 0);
        aircraft.add(tail);

        return aircraft;
    }
};