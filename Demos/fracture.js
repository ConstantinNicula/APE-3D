
var MAX_BLOCKS = 9;

function Block(){
    // Inherit collision box
    APE.CollisionBox.call(this);

    this.exists = false;
    this.body = new APE.RigidBody();
    this.mesh = null;

}
// Inherit collision box prototype
APE.extend(Block, APE.CollisionBox);


/**
 * Draws the block.
 */
Block.prototype.render = function(){
    if(this.exists){
        if(this.mesh === null){
            this.mesh = new THREE.Mesh(
                new THREE.BoxGeometry(this.halfSize.x*2, this.halfSize.y*2, this.halfSize.z*2),
                new THREE.MeshLambertMaterial({color:(this.body.getAwake()? 0xffb2b2: 0xb2b2ff)})
            );
            scene.add(this.mesh);
            var p = this.body.getPosition();
            var q = this.body.getOrientation();

            this.mesh.position.set(p.x, p.y, p.z);
            this.mesh.quaternion.set(q.i, q.j, q.k, q.r);

        }else{
            var p = this.body.getPosition();
            var q = this.body.getOrientation();

            this.mesh.position.set(p.x, p.y, p.z);
            this.mesh.quaternion.set(q.i, q.j, q.k, q.r);
        }
    }else{
        if(this.mesh){
            this.mesh.visible = false;
        }
    }
};


/**
 * Sets the block to a specific location.
 */
Block.prototype.setState =  function(position, orientation, extents, velocity){
    this.body.setPosition(position);
    this.body.setOrientation(orientation);
    this.body.setVelocity(velocity);
    this.body.setRotation(new APE.Vector3());
    this.halfSize = extents;

    var mass = this.halfSize.x * this.halfSize.y * this.halfSize.z * 8.0;
    this.body.setMass(mass);

    var tensor = new APE.Matrix3();
    tensor.setBlockInertiaTensor(this.halfSize, mass);
    this.body.setInertiaTensor(tensor);

    this.body.setLinearDamping(0.95);
    this.body.setAngularDamping(0.8);
    this.body.setAcceleration(new APE.Vector3(0, -10, 0));

    this.body.setAwake();
    this.body.calculateDerivedData();
};

/**
 * Calculates and sets the mass and inertia tensor of this block,
 * assuming it has the given constant density.
 */
Block.prototype.calculateMassProperties = function(invDensity){
    // Check for infinite mass
    if(invDensity <= 0){
        // Just set zeros for both mass and inertia tensor
        this.body.setInverseMass(0);
        this.body.setInverseInertiaTensor(new APE.Matrix3());
    }else{
        // Otherwise we need to calculate the mass
        var volume = this.halfSize.magnitude() * 2.0;
        var mass = volume/ invDensity;

        this.body.setMass(mass);

        // And calculate the inertia tensor from the mass and size
        mass *= 0.333;
        var tensor = new APE.Matrix3();
        tensor.setInertiaTensorCoeffs(
            mass * this.halfSize.y*this.halfSize.y + this.halfSize.z*this.halfSize.z,
            mass * this.halfSize.y*this.halfSize.x + this.halfSize.z*this.halfSize.z,
            mass * this.halfSize.y*this.halfSize.x + this.halfSize.z*this.halfSize.y
        );

        this.body.setInertiaTensor(tensor);
    }
};

/**
 * Performs the division of the given block into four, writing the
 * eight new blocks into the given blocks array. The blocks array can be
 * a pointer to the same location as the target pointer: since the
 * original block is always deleted, this effectively reuses its storage.
 * The algorithm is structured to allow this reuse.
 */
Block.prototype.divideBlock = function(contact, target, blocks) {
    // Find out if we're body one or two in the contact structure, and
    // therefore what the contact normal is.

    var normal = contact.contactNormal.clone();
    var body = contact.body[0];
    if (body !== target.body) {
        normal.invert();
        body = contact.body[1];
    }


    // Work out where on the body (in body coordinates) the conatct is
    // and its direction.
    var point = body.getPoinInLocalSpace(contact.contactPoint);
    normal = body.getDirectionInLocalSpace(normal);

    // Work out the centre of the split: this is the point coordinates
    // for each of the axes perpendicular to the normal, and 0 for the
    // axis along the normal.

    point = point.sub(normal.multiplyScalar(point.dot(normal)));

    // Take a copy of the half size, so we ca create the new blocks.
    var size = target.halfSize.clone();


    // Take a copy also of the body's other data.
    var tempBody = new APE.RigidBody();
    tempBody.setPosition(this.body.getPosition());
    tempBody.setOrientation(this.body.getOrientation());
    tempBody.setVelocity(this.body.getVelocity());
    tempBody.setRotation(this.body.getRotation());
    tempBody.setLinearDamping(this.body.getAngularDamping());
    tempBody.setAngularDamping(this.body.getAngularDamping());
    tempBody.setInverseInertiaTensor(this.body.getInverseInertiaTensor());
    tempBody.calculateDerivedData();

    // Remove the old block
    target.exists = false;

    //Work out the inverse density of the old block
    var invDensity = this.halfSize.magnitude() * 8 * body.getInverseMass();

    // Now split the block int eight.
    for (var i = 0; i < 8; i++) {
        // Find the minimum and maximum extents of the new block
        // in old-block coordinates.

        var min = new APE.Vector3(),
            max = new APE.Vector3();

        if ((i & 1) == 0) {
            min.x = -size.x;
            max.x = point.x;
        } else {
            min.x = point.x;
            max.x = size.x;
        }
        if ((i & 2) == 0) {
            min.y = -size.y;
            max.y = point.y;
        } else {
            min.y = point.y;
            max.y = size.y;
        }
        if ((i & 4) == 0) {
            min.z = -size.z;
            max.z = point.z;
        } else {
            min.z = point.z;
            max.z = size.z;
        }


        // Get the origin and half size of the block, in old-body
        // local coordinates.
        var halfSize = max.sub(min).multiplyScalar(0.5);
        var newPos = halfSize.add(min);

        // Convert the origin to world coordinates.
        newPos = tempBody.getPointInWorldSpace(newPos);

        // Work out the direction to the impact.
        var direction = newPos.sub(contact.contactPoint);
        direction.normalize();


        // Set the body's properties (we assume the block has a body
        // already that we're going to overwrite).
        blocks[i + 1].body.setPosition(newPos);
        blocks[i + 1].body.setVelocity(tempBody.getVelocity().add(direction.multiplyScalar(10)));
        blocks[i + 1].body.setOrientation(tempBody.getOrientation());
        blocks[i + 1].body.setRotation(tempBody.getRotation());
        blocks[i + 1].body.setLinearDamping(tempBody.getLinearDamping());
        blocks[i + 1].body.setAngularDamping(tempBody.getAngularDamping());
        blocks[i + 1].body.setAwake(true);
        blocks[i + 1].body.setAcceleration(APE.GRAVITY);
        blocks[i + 1].body.clearAccumulators();
        blocks[i + 1].body.calculateDerivedData();
        blocks[i + 1].offset = new APE.Matrix4();
        blocks[i + 1].exists = true;
        blocks[i + 1].halfSize = halfSize.clone();

        // Finally calculate the mass and inertia tensor of the new block.
        blocks[i + 1].calculateMassProperties(invDensity);

    }
};

/**
 * The main demo class definition.
 */

function FractureDemo(){
    /**
     * Track if a block has been hit.
     */
    this.hit  = false;
    this.ball_active = true;
    this.fracture_contact = 0;

    this.maxContacts = 256;
    this.contacts = [];
    this.cData = new APE.CollisionData();
    this.resolver =  new APE.ContactResolver(this.maxContacts*8, this.maxContacts*8);
    this.cData.contactArray = this.contacts;
    this.cData.reset(this.maxContacts);
    /**
     * Holds the bodies.
     */
    this.blocks = [];
    for(var i = 0; i<MAX_BLOCKS; i++){
        this.blocks.push(new Block());
    }
    /**
     * Holds the projectile.
     */
    this.ball = new APE.CollisionSphere();
    this.ball.body = new APE.RigidBody();
    this.ball.radius = 0.25;
    this.ball.body.setMass(5.0);
    this.ball.body.setDamping(0.9, 0.9);
    var it = new APE.Matrix3();
    it.setDiagonal(5.0, 5.0, 5.0);
    this.ball.body.setInertiaTensor(it);
    this.ball.body.setAcceleration(APE.GRAVITY);

    this.ball.body.setCanSleep(false);
    this.ball.body.setAwake(true);


    this.ballMesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.25, 16, 8),
        new THREE.MeshLambertMaterial({color: 0x66b266})
    );
    scene.add(this.ballMesh);
    // Set up the initial block
    this.reset();
}

FractureDemo.prototype = {
    constructor: FractureDemo,

    generateContacts: function(){
        this.hit = false;

        // Create the ground plane data
        var plane = new APE.CollisionPlane();
        plane.direction = new APE.Vector3(0, 1, 0);
        plane.offset = 0;

        // set up the collision data structure
        this.cData.reset(this.maxContacts);
        this.cData.friction = 0.9;
        this.cData.restitution = 0.2;
        this.cData.tolerance = 0.1;

        // Perform collision detection.
        for(var i = 0; i < MAX_BLOCKS; i++){
            if(!this.blocks[i].exists){
                continue;
            }
            // Check for collisions with the ground plane.
            if(!this.cData.hasMoreContacts()){
                return;
            }
            APE.CollisionDetector.boxAndHalfSpace(this.blocks[i], plane, this.cData);
            if(this.ball_active){
                // And with the sphere
                if(!this.cData.hasMoreContacts()){
                    return;
                }

                if(APE.CollisionDetector.boxAndSphere(this.blocks[i], this.ball, this.cData )){
                    this.hit = true;
                    this.fracture_contact = this.cData.contactArray[this.cData.contactArray.length - 1];
                    console.log(this.fracture_contact);
                }
            }

            // Check for collisions with each other box
            for(var j = i+1; j< MAX_BLOCKS; j++ ){
                if(!this.blocks[j].exists){
                    continue;
                }

                if(!this.cData.hasMoreContacts()){
                    return;
                }

                APE.CollisionDetector.boxAndBox(this.blocks[i], this.blocks[j], this.cData);
            }
        }

        // Check for sphere ground collisions
        if(this.ball_active){
            if(!this.cData.hasMoreContacts()){
                return;
            }
            APE.CollisionDetector.sphereAndHalfSpace(this.ball, plane, this.cData);
        }
    },

    reset: function(){
        // Only the first block exists
        this.blocks[0].exists =  true;
        for(var i = 1; i< MAX_BLOCKS; i++){
            this.blocks[i].exists = false;
        }

        // Set the first block
        this.blocks[0].halfSize = new APE.Vector3(4,4,4);
        this.blocks[0].body.setPosition(new APE.Vector3(0, 7, 0));
        this.blocks[0].body.setOrientation(new APE.Quaternion());
        this.blocks[0].body.setVelocity(new APE.Vector3(0,0,0));
        this.blocks[0].body.setRotation(new APE.Vector3(0,0,0));
        this.blocks[0].body.setMass(100);

        var it = new APE.Matrix3();
        it.setBlockInertiaTensor(this.blocks[0].halfSize, 100);
        this.blocks[0].body.setInertiaTensor(it);
        this.blocks[0].body.setDamping(0.9,0.9);
        this.blocks[0].body.calculateDerivedData();
        this.blocks[0].calculateInternals();

        this.blocks[0].body.setAcceleration(APE.GRAVITY);
        this.blocks[0].body.setAwake(true);
        this.blocks[0].body.setCanSleep(true);

        this.ball_active = true;

        // Set up the ball
        this.ball.body.setPosition(new APE.Vector3(0, 5, 20));
        this.ball.body.setOrientation(new APE.Quaternion());
        this.ball.body.setVelocity(
            new APE.Vector3(
                Math.random() * 4 - Math.random() * 4,
                1 + Math.random()*5,
                -20
            )
        );
        this.ball.body.setRotation(new APE.Vector3(0,0,0));
        this.ball.body.calculateDerivedData();
        this.ball.body.setAwake(true);
        this.ball.calculateInternals();

        this.hit = false;

        // Reset the conatcts
        this.cData.reset(this.maxContacts);
    },

    update: function(){
        var duration = 1/60;
        for(var i = 0; i<MAX_BLOCKS; i++){
            if(this.blocks[i].exists){
                this.blocks[i].body.integrate(duration);
                this.blocks[i].calculateInternals();
            }
        }

        if(this.ball_active){
            this.ball.body.integrate(duration);
            this.ball.calculateInternals();
        }

        this.generateContacts();

        this.resolver.resolveContacts(
            this.cData.contactArray,
            this.cData.contactArray.length,
            duration
        );


        // Handle fractures.
        if(this.hit){
            this.blocks[0].divideBlock(
                this.fracture_contact,
                this.blocks[0],
                this.blocks
            );

            this.ball_active = false;
        }

    },

    render: function(){
        // render ball if active
        var p, q;
        if(this.ball_active){
            p = this.ball.body.getPosition();
            q = this.ball.body.getOrientation();
            this.ballMesh.position.set(p.x, p.y, p.z);
            this.ballMesh.quaternion.set(q.i, q.j, q.k, q.r);
        }else{
            this.ballMesh.visible = false;
        }

        for(var i = 0; i < MAX_BLOCKS; i++){
            this.blocks[i].render();
        }
    }


};