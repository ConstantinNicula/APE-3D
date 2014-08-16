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
		
