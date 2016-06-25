module.exports = Ellipsoid;

var Shape = require('./Shape');
var Vec3 = require('../math/Vec3');

/**
 * Ellipsoid shape (reference: https://en.wikipedia.org/wiki/Ellipsoid)
 *
 *   (x / a)^2 + (y / b)^2 + (z / c)^2 = 1
 *
 * @class Ellipsoid
 * @constructor
 * @extends Shape
 * @param {Number} a Length of semi-principal x-axis
 * @param {Number} b Length of semi-principal y-axis
 * @param {Number} c Length of semi-principal z-axis
 * @author jzitelli / http://github.com/jzitelli
 */
function Ellipsoid(a, b, c) {
    Shape.call(this);

    /**
     * @property {Number} a
     */
    this.a = a !== undefined ? Number(a) : 1.0;
    /**
     * @property {Number} b
     */
    this.b = b !== undefined ? Number(b) : 1.0;
    /**
     * @property {Number} c
     */
    this.c = c !== undefined ? Number(c) : 1.0;

    this.type = Shape.types.ELLIPSOID;

    if(this.a <= 0 || this.b <= 0 || this.c <= 0){
        throw new Error('The Ellipsoid lengths must be positive.');
    }

    this.updateBoundingSphereRadius();
}
Ellipsoid.prototype = new Shape();
Ellipsoid.prototype.constructor = Ellipsoid;

Ellipsoid.prototype.calculateLocalInertia = function(mass,target){
    target = target || new Vec3();
    var a = this.a,
        b = this.b,
        c = this.c;
    target.x = mass * (b*b + c*c) / 5.0;
    target.y = mass * (a*a + c*c) / 5.0;
    target.z = mass * (a*a + b*b) / 5.0;
    return target;
};

Ellipsoid.prototype.volume = function(){
    return 4.0 * Math.PI * this.a * this.b * this.c / 3.0;
};

Ellipsoid.prototype.updateBoundingSphereRadius = function(){
    this.boundingSphereRadius = Math.max(this.a, this.b, this.c);
};

var rotatedLengthsTemp = new Vec3();
Ellipsoid.prototype.calculateWorldAABB = function(pos,quat,min,max) {
    /**
     * Points \vec{x} := [x_1, x_2, x_3]^T on the rotated ellipsoid (ignoring translation for now) satisfy the equation
     *
     *   f(\vec{x}) := \vec{x}^T A \vec{x} = f(\vec{x})
     *               = \vec{x}^T U \Sigma U^T \vec{x} = 1,
     *
     * where U is the 3x3 orthogonal rotation matrix (computed from the quaternion),
     * \Sigma is the diagonal matrix of eigenvalues (1/a^2, 1/b^2, 1/c^2).
     *
     * The two points attaining min/max along the (e.g.) x-axis occur where
     *
     *   \grad{f}(\vec{x})_2 = 0, \grad{f}(\vec{x})_3 = 0.
     *
     * Expression for the gradient:
     *
     *   \grad{f}(\vec{x}) = 2 U \Sigma U^T \vec{x}.
     *
     * Solving for a given RHS \vec{b}:
     *
     *   \grad{f}(\vec{x}) = \vec{b}
     *      ===>  \vec{x}  = (1/2) U \Sigma^{-1} U^T \vec{b}.
     *
     * For \vec{b} := [b_1, 0, 0]^T,
     *
     */

};
