module.exports = ImplicitCylinder;

var Shape = require('./Shape');
var Vec3 = require('../math/Vec3');
var Quaternion = require('../math/Quaternion');

/**
 * @class ImplicitCylinder
 * @constructor
 * @author jzitelli / https://github.com/jzitelli
 * @param {Number} radiusTop
 * @param {Number} radiusBottom
 * @param {Number} height
 */
function ImplicitCylinder( radiusTop, radiusBottom, height ) {
    Shape.call(this);
    this.type = Shape.types.IMPLICITCYLINDER;
    this.radiusTop = radiusTop;
    this.radiusBottom = radiusBottom;
    this.height = height;
}

ImplicitCylinder.prototype = new Shape();
ImplicitCylinder.prototype.constructor = ImplicitCylinder;
