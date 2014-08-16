/**
 * Creates the basic context for the APE-3D engine, a lightweight, impulse-based,
 * mass aggregate physics engine. APE-3D stands for Another Physics Engine.
 */

var APE = { VERSION : 'Alpha-2' };

APE.extend = function(child, parent){
    function F(){}
    F.prototype = parent.prototype;
    child.prototype =  new F();
    child.prototype.constructor = child;
};

/**
 * Holds a vector in three dimensions.
 */

APE.Vector3 = function(x, y, z){
    /** Holds the value along the x axis. */
    this.x = x || 0;
    /** Holds the value along the y axis. */
    this.y = y || 0;
    /** Holds the value along the z axis. */
    this.z = z || 0;
};


APE.Vector3.prototype = {

    constructor: APE.Vector3,

    /** Flips all the components of the vector. */
    invert : function(){
        this.x = -this.x;
        this.y = -this.y;
        this.z = -this.z;
    },

    /** Gets the magnitude of this vector. */
    magnitude : function(){
        return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
    },

    /** Gets the squared magnitude of this vector. */
    squareMagnitude : function(){
        return this.x*this.x + this.y*this.y + this.z*this.z;
    },

    /** Turns a non-zero vector into a vector of unit length. */
    normalize : function(){
        var l = this.magnitude();
        if(l > 0){
            this.x *= 1/l;
            this.y *= 1/l;
            this.z *= 1/l;
        }
    },

    /** Returns the normalised version of this vector. */
    unit: function(){
        var result = this.clone();
        result.normalize();
        return  result;
    },


    /** Multiplies this vector by the given scalar. */
    multiplyScalar : function(value){
        return  new APE.Vector3(this.x * value,
                this.y * value,
                this.z * value);
    },

    /** Adds the given vector to this. */
    add : function(v){
        return new APE.Vector3( this.x + v.x,
                this.y + v.y,
                this.z + v.z);
    },


    /** Subtracts the given vector from this. */
    sub : function(v){
        return new APE.Vector3( this.x - v.x,
                this.y - v.y,
                this.z - v.z);
    },

    /** Adds the given vector to this, scaled by the given amount. */
    addScaledVector : function (vector, scale){
        this.x += vector.x * scale;
        this.y += vector.y * scale;
        this.z += vector.z * scale;
    },


    /**
     * Calculates and returns a component-wise product of this
     * vector with the given vector.
     */
    componentProduct: function(vector){
        return new APE.Vector3(this.x * vector.x,
                this.y * vector.y,
                this.z * vector.z);
    },

    /**
     * Calculates and returns the scalar product of this vector
     * with the given vector.
     */
    dot : function(vector){
        return this.x * vector.x + this.y * vector.y  + this.z * vector.z;
    },

    /**
     * Calculates and returns the vector product of this vector
     * with the given vector.
     */
    cross : function(vector){
        return new APE.Vector3( this.y * vector.z - this.z * vector.y,
                this.z * vector.x - this.x * vector.z,
                this.x * vector.y - this.y * vector.x);
    },

    /**
     * Returns a copy of the given vector.
     */
    clone : function(){
        return new APE.Vector3(this.x, this.y, this.z);
    },

    /**
     * Zero all the components of the vector.
     */
    clear: function(){
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }
};
/**
 * Calculate an orthonormal basis based on three
 * vector values.
 */

function makeOrthonormalBasis(a, b, c){
    a.normalize();
    c = a.cross(b);
    if(c.squareMagnitude() === 0.0){
        return;
    }
    c.normalize();
    b = c.cross(a);
    return [a,b,c];
}

/**
 * Holds an inertia tensor, consisting of a 3x3 row-major matrix.
 * This matrix is not padding to produce an aligned structure, since
 * it is most commonly used with a mass (single real) and two
 * damping coefficients to make the 12 -  element characteristics array
 * of a rigid body.
 */

/**
 * Creates a new matrix.
 */
APE.Matrix3 = function(c0, c1, c2, c3, c4, c5, c6, c7, c8){
    /**
     * Initialize the matrix components.
     */
    c0 = (c0 !== undefined)? c0 : 1 ;
    c1 = c1 || 0;
    c2 = c2 || 0;

    c3 = c3 || 0;
    c4 = (c4 !== undefined)? c4 : 1;
    c5 = c5 || 0;

    c6 = c6 || 0;
    c7 = c7 || 0;
    c8 = (c8 !== undefined)? c8 : 1;

    /**
     * Holds the tensor matrix data in array form.
     */

    this.data = [c0, c1, c2,
        c3, c4, c5,
        c6, c7, c8];
};

APE.Matrix3.prototype = {
    constructor: APE.Matrix3,

    /**
     * Transform the given vector by this matrix.
     */

    transform: function(vector){
        return new APE.Vector3(
                vector.x * this.data[0] + vector.y * this.data[1] + vector.z * this.data[2],
                vector.x * this.data[3] + vector.y * this.data[4] + vector.z * this.data[5],
                vector.x * this.data[6] + vector.y * this.data[7] + vector.z * this.data[8]
        );
    },

    /**
     * Multiplies this matrix in by the given matrix and returns
     * a new matrix
     */
    multiply: function(o){
        return new APE.Matrix3(
                this.data[0] * o.data[0] + this.data[1] * o.data[3] + this.data[2] * o.data[6],
                this.data[0] * o.data[1] + this.data[1] * o.data[4] + this.data[2] * o.data[7],
                this.data[0] * o.data[2] + this.data[1] * o.data[5] + this.data[2] * o.data[8],

                this.data[3] * o.data[0] + this.data[4] * o.data[3] + this.data[5] * o.data[6],
                this.data[3] * o.data[1] + this.data[4] * o.data[4] + this.data[5] * o.data[7],
                this.data[3] * o.data[2] + this.data[4] * o.data[5] + this.data[5] * o.data[8],

                this.data[6] * o.data[0] + this.data[7] * o.data[3] + this.data[8] * o.data[6],
                this.data[6] * o.data[1] + this.data[7] * o.data[4] + this.data[8] * o.data[7],
                this.data[6] * o.data[2] + this.data[7] * o.data[5] + this.data[8] * o.data[8]
        );
    },

    /**
     * Sets the matrix to be the inverse of the given matrix.
     */
    setInverse : function(m){
        var t4 = m.data[0] * m.data[4],
            t6 = m.data[0] * m.data[5],
            t8 = m.data[1] * m.data[3],
            t10 = m.data[2] * m.data[3],
            t12 = m.data[1] * m.data[6],
            t14 = m.data[2] * m.data[6];

        //Calculate the determinant
        var t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
            t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

        // Make sure the determinant is non-zero.
        if(t16 === 0.0){
            return;
        }
        var t17 = 1/t16;

        this.data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
        this.data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
        this.data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
        this.data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
        this.data[4] = (m.data[0] * m.data[8] - t14) * t17;
        this.data[5] = -(t6 - t10) * t17;
        this.data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
        this.data[7] = -(m.data[0] * m.data[7] - t12) *t17;
        this.data[8] = (t4 - t8) * t17;
    },

    /** Returns a new matrix containing the inverse of this matrix, */
    inverse : function(){
        var result = new APE.Matrix3();
        result.setInverse(this);
        return result;
    },

    /**
     * Inverts the matrix.
     */
    invert : function(){
        this.setInverse(this.clone());
    },

    /**
     * Sets the matrix to be the transpose of the given matrix.
     */
    setTranspose: function(m){
        this.data[0] = m.data[0];
        this.data[1] = m.data[3];
        this.data[2] = m.data[6];
        this.data[3] = m.data[1];
        this.data[4] = m.data[4];
        this.data[5] = m.data[7];
        this.data[6] = m.data[2];
        this.data[7] = m.data[5];
        this.data[8] = m.data[8];
    },

    /**
     * Returns a new matrix containing the transpose of this matrix.
     */
    transpose: function(){
        var result = new APE.Matrix3();
        result.setTranspose(this);
        return result;
    },

    /**
     * Sets this matrix to be the rotation matrix corresponding to
     * the given quaternion.
     */
    setOrientation: function(q){
        this.data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k );
        this.data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        this.data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
        this.data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
        this.data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        this.data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
        this.data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
        this.data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
        this.data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
    },

    clone: function(){
        var result = new APE.Matrix3();
        result.data[0] = this.data[0];
        result.data[1] = this.data[1];
        result.data[2] = this.data[2];

        result.data[3] = this.data[3];
        result.data[4] = this.data[4];
        result.data[5] = this.data[5];

        result.data[6] = this.data[6];
        result.data[7] = this.data[7];
        result.data[8] = this.data[8];
        return result;
    },

    /**
     * Sets the matrix to be diagonal matrix with the given
     * values along the leading diagonal.
     */
    setDiagonal: function(a, b, c){
        this.setInertiaTensorCoeffs(a ,b, c);
    },

    /**
     * Sets the matrix to be a diagonal matrix with the given
     * values along the leading diagonal.
     */
    setInertiaTensorCoeffs: function(ix, iy, iz, ixy, ixz, iyz){
        ixy = ixy || 0;
        ixz = ixz || 0;
        iyz = iyz || 0;

        this.data[0] = ix;
        this.data[1] = this.data[3] = -ixy;
        this.data[2] = this.data[6] = -ixz;
        this.data[4] = iy;
        this.data[5] = this.data[7] = -iyz;
        this.data[8] = iz;
    },

    /**
     * Sets the value of the matrix as an inertia tensor of
     * a rectangular block aligned with the body's coordinate
     * system with the given axis half-sizes and mass.
     */
    setBlockInertiaTensor: function(halfSizes, mass){
        var squares = halfSizes.componentProduct(halfSizes);
        this.setInertiaTensorCoeffs(
                0.3 * mass * (squares.y + squares.z),
                0.3 * mass * (squares.x + squares.z),
                0.3 * mass * (squares.x + squares.y)
        );
    },

    /**
     * Sets the matrix to be a skew symmetric matrix based on
     * the given vector. The skew symmetric matrix is the
     * equivalent of the vector product. So if a,b are vectors,
     * a x b = A_s * b where A_s is the skew symmetric form of a.
     */
    setSkewSymmetric: function(vector){
        this.data[0] = this.data[4] = this.data[8] = 0;
        this.data[1] = -vector.z;
        this.data[2] = vector.y;
        this.data[3] = vector.z;
        this.data[5] = -vector.x;
        this.data[6] = -vector.y;
        this.data[7] = vector.x;
    },

    /**
     * Set the matrix values from the given three vector components.
     * these are arranged as the three columns of the matrix.
     */
    setComponents: function(compOne, compTwo, compThree){
        this.data[0] = compOne.x;
        this.data[1] = compTwo.x;
        this.data[2] = compThree.x;

        this.data[3] = compOne.y;
        this.data[4] = compTwo.y;
        this.data[5] = compThree.y;

        this.data[6] = compOne.z;
        this.data[7] = compTwo.z;
        this.data[8] = compThree.z;
    },

    /**
     * Transform the given vector by the transpose of this matrix.
     */
    transformTranspose: function(vector){
        return new APE.Vector3(
                vector.x * this.data[0] + vector.y * this.data[3] + vector.z * this.data[6],
                vector.x * this.data[1] + vector.y * this.data[4] + vector.z * this.data[7],
                vector.x * this.data[2] + vector.y * this.data[5] + vector.z * this.data[8]
        );
    },

    /**
     * Gets a vector representing one row in the matrix.
     */
    getRowVector: function(i) {
        return new APE.Vector3(this.data[i * 3], this.data[i * 3 + 1], this.data[i * 3 + 2]);
    },

    /**
     * Gets a vector representing one axis (one column) in the matrix.
     */
    getAxisVector: function(i){
        return new APE.Vector3(this.data[i], this.data[i+3], this.data[i+6]);
    },

    /**
     * Multiplies each component of the matrix by the given scalar.
     */
    multiplyScalar: function(scalar){
        return new APE.Matrix3(
                this.data[0] * scalar, this.data[1] * scalar, this.data[2] * scalar,
                this.data[3] * scalar, this.data[4] * scalar, this.data[5] * scalar,
                this.data[6] * scalar, this.data[7] * scalar, this.data[8] * scalar
        );
    },

    /**
     * Returns a new matrix equal to this matrix puls the given matrix.
     */
    add: function(o){
        return new APE.Matrix3(
                this.data[0] + o.data[0], this.data[1] + o.data[1], this.data[2] + o.data[2],
                this.data[3] + o.data[3], this.data[4] + o.data[4], this.data[5] + o.data[5],
                this.data[6] + o.data[6], this.data[7] + o.data[7], this.data[8] + o.data[8]
        );
    }

};

/**
 * Interpolates a couple of matrices.
 */
APE.Matrix3.linearInterpolate = function(a, b, prop){
    var result = new APE.Matrix3();
    var omp = 1 - prop;
    for(var  i = 0; i<9; i++){
        result.data[i] = a.data[i] * omp + b.data[i] * prop;
    }
    return result;
};


/**
 * Holds a transform matrix, consisting of a rotation matrix and
 * a position. The matrix has 12 elements, it is assumed that the
 * remaining four are (0,0,0,1); producing a homogeneous matrix.
 */
APE.Matrix4 = function(){
    /**
     * Holds the transform matrix data in array form.
     */
    this.data = [1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0];

};

APE.Matrix4.prototype = {
    constructor: APE.Matrix4,

    /**
     * Transform the given vector by this matrix.
     */

    transform: function(vector){
        return new APE.Vector3(
                vector.x * this.data[0] +
                vector.y * this.data[1] +
                vector.z * this.data[2] + this.data[3],

                vector.x * this.data[4] +
                vector.y * this.data[5] +
                vector.z * this.data[6] + this.data[7],

                vector.x * this.data[8] +
                vector.y * this.data[9] +
                vector.z * this.data[10] + this.data[11]
        );
    },

    /**
     * Return a matrix, which is this one multiplied by another given matrix.
     */
    multiply:  function(o){
        var result = new APE.Matrix4();
        result.data[0] =(o.data[0] * this.data[0]) +(o.data[4] * this.data[1]) + (o.data[8] * this.data[2]);
        result.data[4] =(o.data[0] * this.data[4]) +(o.data[4] * this.data[5]) + (o.data[8] * this.data[6]);
        result.data[8] =(o.data[0] * this.data[8]) +(o.data[4] * this.data[9]) + (o.data[8] * this.data[10]);

        result.data[1] =(o.data[1] * this.data[0]) +(o.data[5] * this.data[1]) + (o.data[9] * this.data[2]);
        result.data[5] =(o.data[1] * this.data[4]) +(o.data[5] * this.data[5]) + (o.data[9] * this.data[6]);
        result.data[9] =(o.data[1] * this.data[8]) +(o.data[5] * this.data[9]) + (o.data[9] * this.data[10]);

        result.data[2] =(o.data[2] * this.data[0]) +(o.data[6] * this.data[1]) + (o.data[10] * this.data[2]);
        result.data[6] =(o.data[2] * this.data[4]) +(o.data[6] * this.data[5]) + (o.data[10] * this.data[6]);
        result.data[10] =(o.data[2] * this.data[8]) +(o.data[6] * this.data[9]) + (o.data[10] * this.data[10]);

        result.data[3] =(o.data[3] * this.data[0]) +(o.data[7] * this.data[1]) + (o.data[11] * this.data[2]) + this.data[3];
        result.data[7] =(o.data[3] * this.data[4]) +(o.data[7] * this.data[5]) + (o.data[11] * this.data[6]) + this.data[7];
        result.data[11] =(o.data[3] * this.data[8]) +(o.data[7] * this.data[9]) + (o.data[11] * this.data[10]) + this.data[11];

        return result;
    },

    /**
     * Return the determinant of the matrix.
     */
    getDeterminant : function(){
        return -this.data[8] * this.data[5] * this.data[2] +
            this.data[4] * this.data[9] * this.data[2] +
            this.data[8] * this.data[1] * this.data[6] -
            this.data[0] * this.data[9] * this.data[6] -
            this.data[4] * this.data[1] * this.data[10] +
            this.data[0] * this.data[5] * this.data[10];
    },

    /**
     * Sets the matrix to the inverse of the given matrix.
     */
    setInverse: function(m){
        var det  =  m.getDeterminant();
        if(det === 0.0){
            return;
        }
        det = 1/det;

        this.data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
        this.data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
        this.data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * det;

        this.data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
        this.data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
        this.data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * det;

        this.data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * det;
        this.data[6] = (m.data[4] * m.data[2] - m.data[0] * m.data[6]) * det;
        this.data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * det;

        this.data[3] = (m.data[9] * m.data[6] * m.data[3]
            -m.data[5] * m.data[10] * m.data[3]
            -m.data[9] * m.data[2] * m.data[7]
            +m.data[1] * m.data[10] * m.data[7]
            +m.data[5] * m.data[2] * m.data[11]
            -m.data[1] * m.data[6] * m.data[11]) * det;

        this.data[7] = (-m.data[8] * m.data[6] * m.data[3]
            +m.data[4] * m.data[10] * m.data[3]
            +m.data[8] * m.data[2] * m.data[7]
            -m.data[0] * m.data[10] * m.data[7]
            -m.data[4] * m.data[2] * m.data[11]
            +m.data[0] * m.data[6] * m.data[11]) * det;

        this.data[11] = (m.data[8] * m.data[5] * m.data[3]
            -m.data[4] * m.data[9] * m.data[3]
            -m.data[8] * m.data[1] * m.data[7]
            +m.data[0] * m.data[9] * m.data[7]
            +m.data[4] * m.data[1] * m.data[11]
            -m.data[0] * m.data[5] * m.data[11]) * det;

    },

    /** Returns a new matrix containing the inverse of this matrix. */
    inverse : function(){
        var result = new APE.Matrix4();
        result.setInverse(this);
        return result;
    },

    /**
     * Inverts the matrix.
     */

    invert : function(){
        this.setInverse(this.clone());
    },

    /**
     * Sets this matrix to be the rotation matrix corresponding to
     * the given quaternion and the position to the input vector.
     */
    setOrientationAndPos: function(q, pos){
        this.data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
        this.data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        this.data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
        this.data[3] = pos.x;

        this.data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
        this.data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        this.data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
        this.data[7] = pos.y;

        this.data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
        this.data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
        this.data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        this.data[11] = pos.z;
    },

    /**
     * Transform the given vector by the transformational inverse
     * of this matrix.
     *
     * This function relies on the fact that the inverse of a
     * pure rotation matrix is its transpose. It separates the
     * translational and rotation components, transposes the
     * rotation and multiplies out. If the matrix is not a scale and shear free
     * transformation matrix, then this function will not give the correct results.
     */

    transformInverse: function(vector){
        var tmp = vector.clone();
        tmp.x -= this.data[3];
        tmp.y -= this.data[7];
        tmp.z -= this.data[11];

        return new APE.Vector3(
                tmp.x * this.data[0] +
                tmp.y * this.data[4] +
                tmp.z * this.data[8],

                tmp.x * this.data[1] +
                tmp.y * this.data[5] +
                tmp.z * this.data[9],

                tmp.x * this.data[2] +
                tmp.y * this.data[6] +
                tmp.z * this.data[10]
        );
    },

    /**
     * Transform the given direction vector by this matrix
     * When a direction is converted between frames of
     * reference, there is no translation required.
     */

    transformDirection: function(vector){
        return new APE.Vector3(
                vector.x * this.data[0] +
                vector.y * this.data[1] +
                vector.z * this.data[2] ,

                vector.x * this.data[4] +
                vector.y * this.data[5] +
                vector.z * this.data[6] ,

                vector.x * this.data[8] +
                vector.y * this.data[9] +
                vector.z * this.data[10]
        );
    },

    /**
     * Transform the given direction vector by the
     * transformational inverse of this matrix.
     *
     * This function relies on the fact the inverse of
     * a pure rotation matrix is its transpose. IT separates the
     * translational and rotation components, transposes the rotation,
     * and multiplies out. If the matrix is not a scale and sheer free
     * transformation matrix, then this function
     * will not give correct results.
     */

    transformInverseDirection: function(vector){
        return new APE.Vector3(
                vector.x * this.data[0] +
                vector.y * this.data[4] +
                vector.z * this.data[8],

                vector.x * this.data[1] +
                vector.y * this.data[5] +
                vector.z * this.data[9],

                vector.x * this.data[2] +
                vector.y * this.data[6] +
                vector.z * this.data[10]
        );
    },

    clone: function(){
        var result = new APE.Matrix4();

        result.data[0] = this.data[0];
        result.data[1] = this.data[1];
        result.data[2] = this.data[2];
        result.data[3] = this.data[3];

        result.data[4] = this.data[4];
        result.data[5] = this.data[5];
        result.data[6] = this.data[6];
        result.data[7] = this.data[7];

        result.data[8] = this.data[8];
        result.data[9] = this.data[9];
        result.data[10] = this.data[10];
        result.data[11] = this.data[11];

        return result;
    },

    /**
     * Gets a vector representing one axis (one column) in the matrix.
     */
    getAxisVector: function(i){
        return new APE.Vector3(this.data[i], this.data[i+4], this.data[i+8]);
    },

    /**
     * Sets the matrix to be a diagonal matrix with the given coefficients.
     */
    setDiagonal : function (a, b, c){
        this.data[0] = a;
        this.data[5] = b;
        this.data[10] = c;
    }

};

/**
 * Holds a three degree of freedom orientation.
 *
 * Quaternions have several mathematical properties that make the useful for
 * representing orientations, but require four items of data to hold
 * the three degrees of freedom, These four items of data can
 * be viewed as the coefficients of a complex numbers with three
 * imaginary parts. The mathematics of the quaternion is the defined an roughly
 * correspondent to the maths of 3D rotations. A quaternion is only valid rotation
 * if it is normalized.
 *
 * Angular velocity and acceleration can correctly be represented as vectors.
 * Quaternions are only needed for orientation.
 */

APE.Quaternion = function(r, i, j, k){
    /**
     * Holds the real component of the quaternion.
     */
    this.r = (r !== undefined)? r : 1;

    /**
     * Holds the first complex component of the
     * quaternion.
     */
    this.i = i || 0;

    /**
     * Holds the second complex component of the
     * quaternion.
     */
    this.j = j || 0;

    /**
     * Holds the third complex component of the
     * quaternion.
     */
    this.k = k || 0;

    /**
     * Holds the quaternion data in array form.
     */
    this.data = [];
};

APE.Quaternion.prototype = {
    constructor: APE.Quaternion,

    /**
     * Normalises the quaternion to unit length, making it a valid
     * orientation quaternion.
     */

    normalize: function(){
        var d = this.r * this.r + this.i * this.i + this.j * this.j + this.k * this.k;

        // Check for zero length quaternion, and use the no-rotation
        // quaternion in that case.

        if(d < APE.real_epsilon) {
            this.r = 1;
            return;
        }

        d = 1/Math.sqrt(d);
        this.r *= d;
        this.i *= d;
        this.j *= d;
        this.k *= d;
    },

    /**
     * Multiplies the quaternion by the given quaternion.
     */
    multiply: function(multiplier){
        return new APE.Quaternion(
                this.r*multiplier.r - this.i*multiplier.i -
                this.j*multiplier.j - this.k*multiplier.k,

                this.r*multiplier.i + this.i*multiplier.r +
                this.j*multiplier.k - this.k*multiplier.j,

                this.r*multiplier.j + this.j*multiplier.r +
                this.k*multiplier.i - this.i*multiplier.k,

                this.r*multiplier.k + this.k*multiplier.r +
                this.i*multiplier.j - this.j*multiplier.i
        );
    },

    /**
     * Adds the given vector to this, scaled by the given amount.
     * This is used to update the orientation quaternion by a rotation
     * and time.
     */
    addScaledVector: function(vector, scale){
        var q = new APE.Quaternion(0,
                vector.x * scale,
                vector.y * scale,
                vector.z * scale);
        q = q.multiply(this);
        this.r += q.r * 0.5;
        this.i += q.i * 0.5;
        this.j += q.j * 0.5;
        this.k += q.k * 0.5;
    },

    rotateByVector: function(vector){
        var q = new APE.Quaternion(0, vector.x, vector.y, vector.z);
        q = this.multiply(q);
        this.r = q.r;
        this.i = q.i;
        this.j = q.j;
        this.k = q.k;
    },

    /**
     * Returns a copy of this quaternion.
     */
    clone: function(){
        return new APE.Quaternion(this.r, this.i, this.j, this.k);
    }
};

/**
 * A particle is the simplest object that can be simulated in the
 * physics system.
 */

APE.Particle = function(){
    /**
     * Holds the linear position of the particle in
     * world space.
     */
    this.position = new APE.Vector3();

    /**
     * Holds the linear velocity of the particle in
     * world space.
     */
    this.velocity = new APE.Vector3();

    /**
     * Holds the acceleration of the particle. This value
     * can be used to set acceleration due to gravity
     * (its primary use), or any other constant acceleration.
     */
    this.acceleration = new APE.Vector3();

    /**
     * Holds the amount of damping applied to linear
     * motion. Damping is required to remove energy added
     * through numerical instability in the integrator
     */
    this.damping =  1;

    /**
     * Holds the inverse of the mass of the particle. It
     * is more useful to hold the inverse mass because
     * integration is simpler, and because in real-time
     * simulation it is more useful to have objects with
     * infinite mass(immovable) than zero mass
     *(completely unstable in numerical simulation).
     */
    this.inverseMass = 1;

    /**
     * Holds the accumulated force to be applied at the next
     * simulation iteration only. THis value is zeroed at each
     * integration step.
     */
    this.forceAccum = new APE.Vector3();
};


APE.Particle.prototype = {

    constructor: APE.Particle,

    /**
     * Integrates the particle forward in time by the given amount.
     * This function uses the Newton-Euler integration method, which is a
     * linear approximation to the correct integral. For this reason it
     * may be inaccurate in some cases.
     */


    integrate : function (duration){
        // We don't integrate things with infinite mass.
        if(this.inverseMass <= 0){
            return;
        }

        //Update linear position.
        this.position.addScaledVector(this.velocity, duration);

        // Work out the acceleration from the force.
        var resultingAcc = this.acceleration.clone();
        resultingAcc.addScaledVector(this.forceAccum, this.inverseMass);

        // Update linear velocity from the acceleration
        this.velocity.addScaledVector(resultingAcc, duration);

        // Impose drag.
        this.velocity =  this.velocity.multiplyScalar(Math.pow(this.damping, duration));

        // Clear the forces.
        this.clearAccumulator();
    },

    /**
     * Clears the forces applied to the particle. This will be
     * called automatically after each integration step.
     */

    clearAccumulator : function(){
        this.forceAccum.clear();
    },

    /**
     * Adds the given force to the particle to be applied at the
     * next iteration only
     */
    addForce : function(force){
        this.forceAccum = this.forceAccum.add(force);
    },

    /**
     * Sets the mass of the particle.
     * The new mass may not be zero. Small masses
     * can produce unstable rigid bodies under simulation.
     */

    setMass : function(mass){
        if(mass !== 0){
            this.inverseMass = 1/mass;
        }
    },

    /**
     * Gets the current mass of the particle.
     */

    getMass : function(){
        if(this.inverseMass === 0){
            return Number.MAX_VALUE;
        }else{
            return 1/this.inverseMass;
        }
    },

    /**
     * Sets the inverse mass of the particle.
     * This may be zero, for a body with infinite mass (unmovable).
     */

    setInverseMass :  function(inverseMass){
        this.inverseMass = inverseMass;
    },

    /**
     * Gets the current inverse mass of the particle.
     */

    getInverseMass : function(){
        return this.inverseMass;
    },

    /**
     * Returns true if the mass of the particle is not infinite.
     */

    hasFiniteMass :  function(){
        return this.inverseMass > 0.0;
    },

    /**
     * Sets the damping of the particle.
     */

    setDamping : function(damping){
        this.damping = damping;
    },

    /**
     * Gets the current damping value.
     */

    getDamping : function(){
        return this.damping;
    },

    /**
     * Sets the position of the particle.
     * The values are set on a component basis in order
     * to avoid referencing issues.
     */

    setPosition : function(position){
        this.position.x = position.x;
        this.position.y = position.y;
        this.position.z = position.z;
    },

    /**
     * Gets the position of the particle.
     */

    getPosition : function(){
        return this.position.clone();
    },

    /**
     * Sets the velocity of the particle.
     * The values are set on a component basis in order to
     * avoid referencing issues.
     */

    setVelocity : function(velocity){
        this.velocity.x = velocity.x;
        this.velocity.y = velocity.y;
        this.velocity.z = velocity.z;
    },

    /**
     * Gets the velocity of the particle.
     */

    getVelocity : function(){
        return this.velocity.clone();
    },

    /**
     * Sets the constant acceleration of the particle.
     * The values are set on a component basis in order to avoid
     * referencing issues.
     */

    setAcceleration : function(acceleration){
        this.acceleration.x = acceleration.x;
        this.acceleration.y = acceleration.y;
        this.acceleration.z = acceleration.z;
    },

    /**
     * Gets the acceleration of the particle.
     */

    getAcceleration : function(){
        return this.acceleration.clone();
    }

};


/**
 * A force generator that applies a gravitational force. One instance
 * can be used for multiple particles.
 */

/** Creates the generator with the given acceleration. */

APE.ParticleGravity = function(gravity){
    /** Holds the acceleration due to gravity.*/
    this.gravity = gravity;
};

//Inherit Particle Force Generator
APE.ParticleGravity.prototype = {
    constructor: APE.ParticleGravity,

    /** Applies the gravitational force to the given Particle.*/
    updateForce : function( particle, duration ){
        // Check that we don not have infinite mass.
        if(!particle.hasFiniteMass()){
            return;
        }
        // Apply the mass-scaled force to the particle.
        particle.addForce(this.gravity.multiplyScalar(particle.getMass()));
    }
};


/**
 * A force generator that applies a drag force. One instance
 * can be used for multiple particles.
 */

/** Creates the generator with the given coefficients. */

APE.ParticleDrag = function(k1, k2){
    /** Holds the velocity drag coefficient.*/
    this.k1 = k1;
    /** Holds the velocity squared drag coefficient. */
    this.k2 = k2;
};

APE.ParticleDrag.prototype = {
    constructor: APE.ParticleDrag,

    /** Applies the drag force to the given particle. */
    updateForce: function( particle, duration ){
        var force = particle.getVelocity();

        //Calculate the total drag coefficient.
        var dragCoeff = force.magnitude();
        dragCoeff = this.k1 * dragCoeff + this.k2 * dragCoeff * dragCoeff;

        //Calculate the final force and apply it.
        force.normalize();
        force = force.multiplyScalar(-dragCoeff);
        particle.addForce(force);
    }
};


/**
 * A force generator that applies a spring force.
 */

/** Creates a new spring wit the given parameters*/

APE.ParticleSpring = function(other, springConstant, restLength){
    /** The particle at the other end of the spring. */
    this.other = other;

    /** Holds the spring constant. */
    this.springConstant = springConstant;

    /** Holds the rest length of the spring. */
    this.restLength = restLength;
};

APE.ParticleSpring.prototype = {
    constructor: APE.ParticleSpring,

    /** Applies the spring force to the given particle. */
    updateForce: function(particle, duration){
        // Calculate the vector of the spring.
        var force = particle.getPosition();
        force = force.sub(this.other.getPosition());

        // Calculate the magnitude of the force.
        var magnitude = force.magnitude();
        magnitude = (this.restLength - magnitude) * this.springConstant;

        // Calculate the final force and apply it.
        force.normalize();
        force = force.multiplyScalar(magnitude);
        particle.addForce(force);
    }
};

/**
 * A force generator that applies a spring force, where
 * one end is attached to a fixed point in space.
 */

/** Creates a new spring with the given parameters.*/
APE.ParticleAnchoredSpring = function(anchor, springConstant, restLength){
    /** The location of the anchored end of the spring. */
    this.anchor = anchor;

    /** Holds the spring constant. */
    this.springConstant = springConstant;

    /** Holds the rest length of the spring. */
    this.restLength = restLength;
};


APE.ParticleAnchoredSpring.prototype = {
    constructor: APE.ParticleAnchoredSpring,
    /** Applies the spring force to the given particle. */
    updateForce : function(particle, duartion){
        // Calculate the vector  of the spring.
        var force = particle.getPosition();
        force = force.sub(this.anchor);

        // Calculate the magnitude of the force.
        var magnitude = force.magnitude();
        magnitude = (this.restLength - magnitude) * this.springConstant;

        // Calculate the final force and apply it.
        force.normalize();
        force = force.multiplyScalar(magnitude);
        particle.addForce(force);
    }
};

/**
 * A force generator that applies a spring force only
 * when extended.
 */

/** Creates a new bungee with the given parameters. */
APE.ParticleBungee = function(other, springConstant, restLength){
    /** The particle at the other end of the spring. */
    this.other = other;
    /** Holds the spring constant. */
    this.springConstant = springConstant;
    /**
     * Holds the length of the bungee at the point it begins to
     * generate a force.
     */
    this.restLength = restLength;
};

APE.ParticleBungee.prototype = {
    constructor: APE.ParticleBungee,

    /** Applies the spring force to the given particle. */
    updateForce : function(particle, duration){
        //Calculate the vector of the spring.
        var force = particle.getPosition();
        force = force.sub(this.other.getPosition());

        // Check if the bungee is compressed.
        var magnitude = force.magnitude();
        if(magnitude <= this.restLength){
            return;
        }

        // Calculate the magnitude of the force.
        magnitude = this.springConstant * (this.restLength - magnitude);

        // Calculate the final force and apply it.
        force.normalize();
        force = force.multiplyScalar(magnitude);
        particle.addForce(force);
    }
};

/**
 * A force generator that applies a buoyancy for a plane of
 * liquid parallel to XZ plane.
 */

/** Creates a new buoyancy force with the given parameters. */
APE.ParticleBuoyancy = function(maxDepth, volume, waterHeight, liquidDensity){
    /**
     * The maximum depth of the object before it generates
     * its maximum buoyancy force.
     */
    this.maxDepth = maxDepth;

    /**
     * The volume of the object.
     */
    this.volume = volume;

    /**
     * The height of the water plane above y = 0. The plane will be
     * parallel to XZ plane.
     */
    this.waterHeight =  waterHeight;

    /**
     * The density of the liquid. Pure water has a density
     * of 1000 kg per cubic meter.
     */
    this.liquidDensity = liquidDensity || 1000;
};

APE.ParticleBuoyancy.prototype = {
    constructor: APE.ParticleBuoyancy,

    /** Applies the buoyancy force to the given particle. */
    updateForce : function(particle, duration){
        // Calculate the submersion depth;
        var depth = particle.getPosition().y;

        //Check if we're out of the water.
        if(depth >= this.waterHeight + this.maxDepth){
            return;
        }

        var force = new APE.Vector3();
        // Check if we're at maximum depth.
        if(depth <= this.waterHeight - this.maxDepth){
            force.y = this.liquidDensity * this.volume;
            particle.addForce(force);
            return ;
        }

        // Otherwise we are partly submerged.
        force.y = this.liquidDensity * this.volume * (this.waterHeight - depth + this.maxDepth)/ ( 2 * this.maxDepth);
        particle.addForce(force);
    }
};

/**
 * A force generator that fakes a stiff spring force, and where
 * one end is attached to a fixed point in space.
 */

/** Creates a new spring with the given parameters. */
APE.ParticleFakeSpring = function(anchor, springConstant, damping){
    /** The location of the anchored end of the spring. */
    this.anchor = new APE.Vector3();

    /** Holds the spring constant. */
    this.springConstant = springConstant;

    /** Holds the damping on the oscillation of the spring. */
    this.damping = damping;
};

APE.ParticleFakeSpring.prototype = {
    constructor: APE.ParticleFakeSpring,

    /** Applies the spring force to the given particle. */
    updateForce: function(particle, duration){
        //Check that we do not have infinite mass.
        if(!particle.hasFiniteMass()){
            return;
        }
        // Calculate the relative position of the particle to the anchor.
        var position = particle.getPosition();
        position = position.sub(this.anchor);

        // Calculate the constants and check that they are in bounds.
        var gamma = 0.5 * Math.sqrt(4 * this.springConstant - this.damping * this.damping);
        if(gamma === 0.0){
            return ;
        }
        var c = new APE.Vector3();
        c = position.multiplyScalar(this.damping/(2 * gamma));
        c = c.add(particle.getVelocity().multiplyScalar(1/ gamma));

        //Calculate the target position.
        var target =  position.multiplyScalar(Math.cos(gamma * duration));
        target = target.add(c.multiplyScalar(Math.sin(gamma * duration)));
        target = target.multiplyScalar(Math.exp(-0.5 * duration * this.damping));


        // Calculates the resulting acceleration, and therefore the force.
        var accel = target.sub(position).multiplyScalar(1/(duration * duration));
        accel = accel.sub(particle.getVelocity().multiplyScalar(1/duration));
        particle.addForce(accel.multiplyScalar(particle.getMass()));
    }
};


/**
 * Keeps track of one force generator and the particle it
 * applies to.
 */

APE.ParticleForceRegistration = function(particle, fg){
    this.particle = particle;
    this.fg = fg;
};

/**
 * Holds all the force generators and the particle that they apply to.
 */

APE.ParticleForceRegistry = function(){
    /**
     * Holds the list of registrations.
     */
    this.registrations = [];
};

APE.ParticleForceRegistry.prototype = {

    constructor: APE.ParticleForceRegistry,

    /**
     * Registers the given force generator to apply to the
     * given particle.
     */
    add : function(particle, fg){
        var registration = new APE.ParticleForceRegistration( particle, fg);
        this.registrations.push(registration);
    },

    /**
     * Removes the given registered pair from the registry.
     * If the pair is not registered, this method will have
     * no effect.
     */
    remove : function(particle, fg){
        var registration, found = false;
        for(var i = 0; i < this.registrations.length && !found; i++){
            registration =  this.registrations[i];
            if(registration.particle === particle && registration.fg === fg){
                this.registrations.splice(i,1);
                found = true;
            }
        }
    },

    /**
     * Clears all registrations from the registry. This will
     * not delete the particles or the force generators
     * themselves, just the records of their connection.
     */
    clear : function(){
        this.registrations = [];
    },

    /**
     * Calls all the force generators to update the forces of their
     * corresponding particles.
     */
    updateForces : function(duration){
        var registration;
        for(var i = 0; i < this.registrations.length; i++){
            registration = this.registrations[i];
            registration.fg.updateForce(registration.particle, duration);
        }
    }
};

/**
 * Links connect two particles together, generating a contact if
 * they violate the constraints of their link. It is used as
 * base class for cables and rods, and could be used as a base
 * class for springs with a limit to their extension.
 */

APE.ParticleLink = function(){
    /**
     * Holds the pair of particles that are connected by this link.
     */
    this.particle = [];
};

APE.ParticleLink.prototype = {
    constructor: APE.ParticleLink,

    /**
     * Returns the current length of the link.
     */

    currentLength : function(){
        var relativePos = this.particle[0].getPosition().sub(this.particle[1].getPosition());
        return relativePos.magnitude();
    },

    /**
     * Generates the contacts to keep this link from being
     * violated. This class can only ever generate a single
     * contact, so the pointer can be a pointer to a single
     * element, the limit parameter is assumed to be at least 1
     * and return the value 0 if the cable wasn't over extended, or
     * 1 if a contact was needed.
     *
     */
    addContact : function( contact, limit){
        // This method will be overloaded in the subclass.
        return 1;
    }
};

/**
 * Cables link a pair of particles, generating a contact if they
 * stray too far apart.
 */
APE.ParticleCable = function(){
    APE.ParticleLink.call(this);
    /**
     * Holds the maximum length of the cable.
     */
    this.maxLength = 0;

    /**
     * Holds the restitution (bounciness) of the cable.
     */
    this.restitution = 1;
};

APE.extend(APE.ParticleCable, APE.ParticleLink);
/**
 * Fills the given contact structure with the contact needed
 * to keep the cable from overExtending.
 */
APE.ParticleCable.prototype.addContact = function(contact, limit){
    // Find the length of the cable.
    var length = this.currentLength();

    //console.log(length);
    // Check if we're overextended.
    if(length < this.maxLength){
        return 0;
    }

    //Otherwise, return the contact.
    contact.particle[0] = this.particle[0];
    contact.particle[1] = this.particle[1];

    // Calculate the normal.
    var normal = this.particle[1].getPosition().sub(this.particle[0].getPosition());
    normal.normalize();
    contact.contactNormal = normal;

    contact.penetration = length - this.maxLength;
    contact.restitution = this.restitution;


    return 1;
};

/**
 * Rods link a pair of particles, generating a contact if they
 * stray too far apart or too close.
 */
APE.ParticleRod = function(){
    APE.ParticleLink.call(this);
    /**
     * Holds the length of the rod.
     */
    this.length = 0;
};

APE.extend(APE.ParticleRod, APE.ParticleLink);

/**
 * Fills the given contact structure with the contact needed
 * to keep the rod from extending or compressing.
 */
APE.ParticleRod.prototype.addContact = function(contact, limit){
    // Find the length of the rod.
    var currentLen = this.currentLength();

    // Check if we're overextended.
    if(currentLen === this.length){
        return 0;
    }

    // Otherwise, return the contact.
    contact.particle[0] = this.particle[0];
    contact.particle[1] = this.particle[1];

    // Calculate the normal.
    var normal = this.particle[1].getPosition().sub(this.particle[0].getPosition());
    normal.normalize();

    // The contact normal depend on whether we're extending or compressing.
    if(currentLen > this.length){
        contact.contactNormal = normal;
        contact.penetration = currentLen - this.length;
    }else{
        contact.contactNormal = normal.multiplyScalar(-1);
        contact.penetration = this.length - currentLen;
    }

    // Always use zero restitution (no bounciness).
    contact.restitution = 0;

    return 1;
};

/**
 * Constraints are just like links, except the connect a particle to
 * an immovable anchor point.
 */

APE.ParticleConstraint = function() {
    /**
     * Holds the particles connected by this constraint.
     */
    this.particle = null;

    /**
     * The point to which the particle is anchored.
     */
    this.anchor = new APE.Vector3();
};

APE.ParticleConstraint.prototype = {
    constructor: APE.ParticleConstraint,

    /**
     * Return the current length of the link.
     */
    currentLength : function(){
        var relativePos = this.particle.getPosition().sub(this.anchor);
        return relativePos.magnitude();
    },
    /**
     * Generates the contacts to keep this link from being
     * violated. This class can only ever generate a single
     * contact, so the pointer can be a pointer to a single
     * element, the limit parameter is assumed to be at least 1
     * and return the value 0 if the cable wasn't over extended, or
     * 1 if a contact was needed.
     *
     */
    addContact : function (contact, limit){
        // To be overloaded in the subclass.
    }
};

/**
 * Cables link a particle to an anchor point, generating a contact if they
 * stray too far apart.
 */

APE.ParticleCableConstraint = function(){
    APE.ParticleConstraint.call(this);

    /**
     * Holds the maximum length of the cable.
     */
    this.maxLength = 0;

    /**
     * Holds the restitution (bounciness) of the cable.
     */
    this.restitution = 1;
};

APE.extend(APE.ParticleCableConstraint, APE.ParticleConstraint);

/**
 * Fills the given contact structure with the contact needed
 * to keep the cable from over-extending.
 */


APE.ParticleCableConstraint.prototype.addContact = function(contact, limit){
    // Find the length of the cable.
    var length = this.currentLength();

    // Check if we're over-extended
    if(length < this.maxLength){
        return 0;
    }

    // Otherwise return the contact
    contact.particle[0] = this.particle;
    contact.particle[1] = null;

    // Calculate the normal
    var normal = this.anchor.sub(this.particle.getPosition());
    normal.normalize();
    contact.contactNormal = normal;

    contact.penetration = length - this.maxLength;
    contact.restitution = this.restitution;

    return 1;
};

/**
 * Rods link particle and to an anchor point, generating a contact if they
 * stray to far apart or too close.
 */

APE.ParticleRodConstraint = function(){
    APE.ParticleConstraint.call(this);

    /**
     * Holds the length of the rod.
     */
    this.length = 0;
};

APE.extend(APE.ParticleRodConstraint, APE.ParticleConstraint);

/**
 * Fills the given contact structure with the contact needed
 * to keep  the rod from extending or compressing.
 */
APE.ParticleRodConstraint.prototype.addContact = function(contact, limit){
    // Find the length of the rod
    var currentLen =  this.currentLength();

    //Check if we're over-extended
    if(currentLen === this.length){
        return 0;
    }

    // Otherwise return the contact.
    contact.particle[0] = this.particle;
    contact.particle[1] = null;

    // Calculate the normal
    var normal = this.anchor.sub(this.particle.getPosition());
    normal.normalize();

    // The contact normal depends on whether we're extending or compressing
    if(currentLen > this.length){
        contact.contactNormal = normal;
        contact.penetration = currentLen - this.length;
    }else{
        contact.contactNormal = normal.multiplyScalar(-1);
        contact.penetration = this.length - currentLen;
    }

    // Always use zero restitution (no bounciness)
    contact.restitution = 0;

    return 1;
};

/**
 * A contact represents two objects in contact (in this case
 * ParticleContact representing two particles). Resolving a
 * contact removes their interpretation, and applies sufficient
 * impulse to keep them apart. Colliding bodies may also rebound.
 *
 * The contact has no callable functions, it just holds the contact details.
 * To resolve a set of contacts, use the contact resolver class.
 */


APE.ParticleContact = function(){
    /**
     * Holds the particles that are involved in the contact. The
     * second of these can be NULL for contacts with the scenery.
     */
    this.particle = [];

    /**
     * Holds the normal restitution coefficient at the contact.
     */
    this.restitution = 1;

    /**
     * Holds the direction of the contact in world coordinates.
     */
    this.contactNormal = new APE.Vector3();

    /**
     * Holds the depth of the penetration at the contact.
     */
    this.penetration = 0;

    /**
     * Holds the amount each particle is moved during interpenetration.
     */
    this.particleMovement = [new APE.Vector3(), new APE.Vector3()];

};

APE.ParticleContact.prototype = {
    constructor: APE.ParticleContact,

    /**
     * Resolves this contact for both velocity and interpenetration.
     */
    resolve : function(duration){
        this.resolveVelocity(duration);
        this.resolveInterpenetration(duration);
    },

    /**
     * Calculates the separating velocity at this contact.
     */
    calculateSeparatingVelocity : function(){
        var relativeVelocity = this.particle[0].getVelocity();
        if(this.particle[1]){
            relativeVelocity = relativeVelocity.sub(this.particle[1].getVelocity());
        }
        return relativeVelocity.dot(this.contactNormal);
    },

    /**
     * Handles the impulse calculations for this collision.
     */
    resolveVelocity : function(duration){
        // Find the velocity in the direction of the contact.
        var separatingVelocity = this.calculateSeparatingVelocity();

        // Check if it needs to be resolved.
        if(separatingVelocity > 0){
            // The contact is either separating, or stationary;
            // no impulse is required.
            return;
        }

        // Calculate the new separating velocity.
        var newSepVelocity = - separatingVelocity * this.restitution;

        // Check the velocity build-up due to acceleration only.
        var accCausedVelocity = this.particle[0].getAcceleration();
        if(this.particle[1]){
            accCausedVelocity = accCausedVelocity.sub(this.particle[1].getAcceleration());
        }
        var accCausedSepVelocity = accCausedVelocity.dot(this.contactNormal) * duration;

        //If we've got a closing velocity due to acceleration build-up,
        // remove it from the new separation velocity.
        if(accCausedSepVelocity < 0){
            newSepVelocity += this.restitution * accCausedSepVelocity;
            // Make sure we haven't removed more than there was
            // to remove;
            if(newSepVelocity < 0){
                newSepVelocity = 0;
            }
        }

        var deltaVelocity = newSepVelocity - separatingVelocity;

        // We apply the change in velocity to each object in proportion to
        // their inverse mass ( those with lower inverse mass [higher actual mass]
        // get less change in velocity).
        var totalInverseMass = this.particle[0].getInverseMass();
        if(this.particle[1]){
            totalInverseMass += this.particle[1].getInverseMass();
        }

        // If all particles have infinite mass, the impulses have no effect.
        if(totalInverseMass <= 0){
            return ;
        }

        // Calculate the impulse to apply.
        var impulse  = deltaVelocity / totalInverseMass;

        // Find the amount of impulse per unit of inverse mass.
        var impulsePerIMass = this.contactNormal.multiplyScalar(impulse);



        // Apply impulses: they are applied in the direction of the contact,
        // and are proportional to the inverse mass.
        this.particle[0].setVelocity(this.particle[0].getVelocity().add(impulsePerIMass.multiplyScalar(this.particle[0].getInverseMass())));
        if(this.particle[1]){
            // particle 1 goes in the opposite direction
            this.particle[1].setVelocity(this.particle[1].getVelocity().add(impulsePerIMass.multiplyScalar(-this.particle[1].getInverseMass())));
        }


    },

    /**
     * Handles the interpenetration resolution for this contact.
     */
    resolveInterpenetration : function(duration){
        // If we don't have any penetration, skip this step.
        if(this.penetration <= 0){
            return;
        }

        // The movement of each of object is based on their mass,
        // so total that.
        var totalInverseMass = this.particle[0].getInverseMass();
        if(this.particle[1]){
            totalInverseMass += this.particle[1].getInverseMass();
        }

        // If all particles have infinite mass, then we do nothing.
        if(totalInverseMass <= 0 ){
            return;
        }

        // Find the amount of penetration resolution per unit
        // of inverse mass.

        var movePerIMass = this.contactNormal.multiplyScalar(this.penetration/ totalInverseMass);

        // Calculate the movement amounts.
        this.particleMovement[0] = movePerIMass.multiplyScalar(this.particle[0].getInverseMass());
        if(this.particle[1]){
            this.particleMovement[1] = movePerIMass.multiplyScalar(-this.particle[1].getInverseMass());
        }else{
            this.particleMovement[1] = new APE.Vector3(0,0,0);
        }

        //Apply the penetration resolution.
        this.particle[0].setPosition(this.particle[0].getPosition().add(this.particleMovement[0]));
        if(this.particle[1]){
            this.particle[1].setPosition(this.particle[1].getPosition().add(this.particleMovement[1]));
        }
    }
};


/**
 * The contact resolution routine for particle contacts. One
 * resolver instance can be shared for the entire simulation.
 */

/** Creates a new contact resolver. */

APE.ParticleContactResolver = function(iterations){
    /**
     * Holds the number of iterations allowed.
     */
    this.iterations = iterations || 0;

    /**
     * This is a performance tracking value; we keep a record
     * of the actual number of iterations used.
     */
    this.iterationsUsed = 0;
};

APE.ParticleContactResolver.prototype = {
    constructor :  APE.ParticleContactResolver,

    /**
     * Sets the number of iterations that can be used.
     */
    setIterations : function(iterations){
        this.iterations = iterations;
    },

    /**
     * Resolves a set of particle contacts for both penetration
     * and velocity.
     */
    resolveContacts :  function(contactArray, numContacts, duration){
        var i;
        this.iterationsUsed = 0;
        while(this.iterationsUsed < this.iterations){
            // Find the contact with the largest closing velocity.
            var max = Number.MAX_VALUE;
            var maxIndex = numContacts;
            for(i = 0; i< numContacts; i++){
                var sepVel = contactArray[i].calculateSeparatingVelocity();
                if(sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0)){
                    max = sepVel;
                    maxIndex = i;
                }
            }

            // Do we have anything worth resolving?
            if(maxIndex === numContacts){
                break;
            }

            //Resolve this contact.
            contactArray[maxIndex].resolve(duration);

            //Update the interpenetration's for all particles

            var move = contactArray[maxIndex].particleMovement;

            for(i = 0; i < numContacts; i++){
                if (contactArray[i].particle[0] === contactArray[maxIndex].particle[0]){
                    contactArray[i].penetration -= move[0].dot(contactArray[i].contactNormal);
                }else if (contactArray[i].particle[0] === contactArray[maxIndex].particle[1]){
                    contactArray[i].penetration -= move[1].dot(contactArray[i].contactNormal);
                }

                if (contactArray[i].particle[1]){
                    if (contactArray[i].particle[1] === contactArray[maxIndex].particle[0]){
                        contactArray[i].penetration += move[0].dot(contactArray[i].contactNormal);
                    }
                    else if (contactArray[i].particle[1] === contactArray[maxIndex].particle[1]){
                        contactArray[i].penetration += move[1].dot(contactArray[i].contactNormal);
                    }
                }
            }
            this.iterationsUsed++;
        }
    }
};

/**
 * Keeps track of a set of particle, and provides means to
 * update them all.
 */

/**
 * Creates a new particle simulator that can handle up to the
 * given number of contacts per frame. You can also optionally
 * give a number of contact-resolution iterations to use. If you
 * don't give a number of iterations, then twice the number of
 * contacts will be used.
 */
APE.ParticleWorld = function(maxContacts, iterations){
    /**
     * Holds the particles.
     */
    this.particles = [];

    /**
     * True if the world should calculate the number of iterations
     * to give the contact resolver at each frame.
     */
    this.calculateIterations = iterations === undefined;

    /**
     * Holds the force generators for the particles in the world.
     */
    this.registry = new APE.ParticleForceRegistry();

    /**
     * Holds the resolver for contacts.
     */
    this.resolver = new APE.ParticleContactResolver(iterations);

    /**
     * Contact generators.
     */
    this.contactGenerators = [];

    /**
     * Holds the list of contacts.
     */
    this.contacts = [];

    /**
     * Holds the maximum number of contacts allowed (the
     * size of the contacts array).
     */
    this.maxContacts = maxContacts;
};

APE.ParticleWorld.prototype = {
    constructor: APE.ParticleWorld,
    /**
     * Initializes the world for a simulation frame. This clears
     * the force accumulators for particles in the world. After
     * calling this, the particles can have their forces for this
     * frame added.
     */
    startFrame : function(){
        for(var i = 0; i < this.particles.length; i++){
            //Remove all forces from the accumulator.
            this.particles[i].clearAccumulator();
        }

        for(var i = 0; i< this.maxContacts; i++){
            this.contacts.push(new APE.ParticleContact());
        }
    },

    /**
     * Calls each of the registered contact generators to report
     * their contacts. Returns the number of generated contacts.
     */
    generateContacts : function(){
        var limit = this.maxContacts;
        var nextContact = 0;
        for(var i = 0; i < this.contactGenerators.length; i++){
            var used;
            if(this.contactGenerators[i] instanceof APE.GroundContacts){
                used = this.contactGenerators[i].addContact(this.contacts, nextContact, limit);
            }else{
                used = this.contactGenerators[i].addContact(this.contacts[nextContact], limit);
            }
            limit -= used;
            nextContact += used;

            // We've run out of contacts to fill. This means we're missing
            // contacts.
            if(limit <= 0){
                break;
            }
        }

        // Return the number of contacts used;
        return this.maxContacts - limit;
    },

    /**
     * Integrates all the particles in this world forward in time
     * by the given duration.
     */
    integrate : function(duration){
        for(var i=0; i < this.particles.length; i++){
            // Integrate the particle by the given duration.
            this.particles[i].integrate(duration);
        }
    },

    /**
     * Processes all the physics for the particle world.
     */
    runPhysics : function(duration){
        // First, apply the force generators.
        this.registry.updateForces(duration);

        // Then integrate the objects.
        this.integrate(duration);

        // Generate contacts.
        var usedContacts = this.generateContacts();

        // And process them.

        if(usedContacts){
            if(this.calculateIterations){
                this.resolver.setIterations(usedContacts * 2);
            }
            this.resolver.resolveContacts(this.contacts, usedContacts, duration);
        }
    },

    /**
     * Returns the list of particles
     */
    getParticles : function(){
        return this.particles;
    },

    /**
     * Get the list of contact generators.
     */
    getContactGenerators : function(){
        return this.contactGenerators;
    },

    /**
     * Returns the force registry.
     */
    getForceRegistry : function(){
        return this.registry;
    }
};

/**
 * A contact generator that takes a vector of particles and
 * collides them against the ground.
 */

APE.GroundContacts = function(){
    this.particles = [];
};

APE.GroundContacts.prototype = {
    constructor : APE.GroundContacts,

    init: function(particles){
        this.particles = particles;
    },

    addContact : function(contacts, nextContact, limit){
        var count = 0;
        for(var i = 0; i < this.particles.length; i++){
            var y = this.particles[i].getPosition().y;
            if(y < 0){
                contacts[nextContact].contactNormal = new APE.Vector3(0, 1, 0);
                contacts[nextContact].particle[0] =  this.particles[i];
                contacts[nextContact].particle[1] = null;
                contacts[nextContact].penetration = -y;
                contacts[nextContact].restitution = 0.2;
                nextContact++;
                count++;
            }
            if(count >= limit){
                return count;
            }
        }
        return count;
    }
};

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
