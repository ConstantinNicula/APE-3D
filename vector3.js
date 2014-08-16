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
