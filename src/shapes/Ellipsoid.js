module.exports = Ellipsoid;

var Shape = require('./Shape');
var Vec3 = require('../math/Vec3');
var Mat3 = require('../math/Mat3');

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

Ellipsoid.prototype.calculateWorldAABB = function(pos,quat,min,max) {
    /**
     * Points \vec{x} := [x_1, x_2, x_3]^T on the rotated ellipsoid (ignoring translation for now) satisfy the equation
     *
     *   f(\vec{x}) := \vec{x}^T A \vec{x} = f(\vec{x})
     *               = \vec{x}^T U \Sigma U^T \vec{x} = 1,
     *
     * where U is the 3x3 orthogonal rotation matrix (determined by the quaternion),
     * \Sigma is the diagonal matrix of eigenvalues [1/(R_1)^2, 1/(R_2)^2, 1/(R_3)^2].
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
     * For \vec{b}_k := [b_1, b_2, b_3]^T defined by
     *
     *   b_k /= 0, b_i = 0 for i /= k,
     *
     * the solution \vec{x} is defined by (in Einstein notation)
     *
     *   x_i = (R_j)^2 U_ij U_kj b_k.
     *
     * Substituting into the ellipsoid eqn:
     *
     *   (b_k)^2 * 1/(R_m)^2 * [(R_j)^2 U_im U_kj U_ij]^2 = 1.
     *
     * Orthogonality annihilates terms, leaving:
     *
     *   (b_k)^2 (R_j)^2 (U_kj)^2 = 1
     *      ===>  b_k = (+/-) [(R_j)^2 (U_kj)^2]^(-1/2).
     *
     * The extremum \vec{x} is then defined by:
     *
     *   x_i = (+/-) (R_j)^2 U_kj U_ij / [(R_m)^2 (U_km)^2]^(1/2).
     */
     var R = new Vec3(this.a, this.b, this.c);
     var R2 = R.vmul(R);
     var U = (new Mat3()).setRotationFromQuaternion(quat);
     var e = U.elements;
     var u0 = new Vec3(e[0], e[1], e[2]);
     var u1 = new Vec3(e[3], e[4], e[5]);
     var u2 = new Vec3(e[6], e[7], e[8]);
     var u02 = u0.vmul(u0);
     var u12 = u1.vmul(u1);
     var u22 = u2.vmul(u2);
     var bInverse = [Math.sqrt(R2.dot(u02)), Math.sqrt(R2.dot(u12)), Math.sqrt(R2.dot(u22))];
     R2 = R2.toArray();
     u02 = u02.toArray();
     u12 = u12.toArray();
     u22 = u22.toArray();
     var x = 0;
     var y = 0;
     var z = 0;
     for (var j = 0; j < 3; j++) {
         x += R2[j] * u02[j] / bInverse[0];
         y += R2[j] * u12[j] / bInverse[1];
         z += R2[j] * u22[j] / bInverse[2];
     }
     max.set(x, y, z);
     min.set(-x, -y, -z);
     max.vadd(pos, max);
     min.vadd(pos, min);
};
