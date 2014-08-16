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