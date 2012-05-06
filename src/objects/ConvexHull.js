/**
 * @class CANNON.ConvexHull
 * @brief A set of points in space describing a convex shape.
 * @author qiao / https://github.com/qiao (original author, see https://github.com/qiao/three.js/commit/85026f0c769e4000148a67d45a9e9b9c5108836f)
 * @author schteppe / https://github.com/schteppe
 * @see http://www.altdevblogaday.com/2011/05/13/contact-generation-between-3d-convex-meshes/
 * @see http://bullet.googlecode.com/svn/trunk/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
 */
CANNON.ConvexHull = function( vertices ) {
  var that = this;
  CANNON.Shape.call( this );

  /**
   * @property array vertices
   * @memberof CANNON.ConvexHull
   * @brief Array of CANNON.Vec3
   */
  this.vertices = [];

  /**
   * @property array faces
   * @memberof CANNON.ConvexHull
   * @brief Array of integer arrays, indicating which vertices each face consists of
   * @todo Needed?
   */
  this.faces = [];

  /**
   * @property array faceNormals
   * @memberof CANNON.ConvexHull
   * @brief Array of CANNON.Vec3
   * @todo Needed?
   */
  this.faceNormals = [];

  /**
   * @property array uniqueEdges
   * @memberof CANNON.ConvexHull
   * @brief Array of CANNON.Vec3
   */
  this.uniqueEdges = [];

  /**
   * @fn addPoints
   * @memberof ConvexHull
   * @brief Add points to the hull
   * @param array points An array of CANNON.Vec3's
   * @param array faces Deprecated - fix autogenerator for these
   * @param array normals Deprecated - fix autogenerator for these
   * @return bool
   * @todo Auto generate faces
   * @todo auto generate normals
   */
  this.addPoints = function( points , faces , normals ) {
    for(pi in points){
      var p = points[pi];
      if(!(p instanceof CANNON.Vec3)){
	throw "Argument 1 must be instance of CANNON.Vec3";
	return false;
      }
      this.vertices.push(p);
    }

    // @todo auto generate. See e.g. https://projects.developer.nokia.com/gles2phys/browser/tags/v1.1.0/src/bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp?rev=22
    this.faces = faces;
    this.faceNormals = normals;
    
    for(var i=0; i<faces.length; i++){
      var numVertices = faces[i].length;
      var NbTris = numVertices;
      for(var j=0; j<NbTris; j++){
	var k = ( j+1 ) % numVertices;
	var edge = new CANNON.Vec3();
	this.vertices[faces[i][j]].vsub(this.vertices[faces[i][k]],edge);
	edge.normalize();
	var found = false;
	for(var p=0;p<this.uniqueEdges.length;p++){
	  if (this.uniqueEdges[p].almostEquals(edge) || 
	      this.uniqueEdges[p].almostEquals(edge)){
	    found = true;
	    break;
	  }
	}

	// What is this for???
	if (!found){
	  this.uniqueEdges.push(edge);
	}

	if (edge) {
	  edge.face1 = i;
	} else {
	  var ed;
	  ed.m_face0 = i;
	  edges.insert(vp,ed);
	}
      }
    }

    return true;
  }

  /**
   * Get max and min dot product of a convex hull at position (pos,quat) projected onto an axis. Results are saved in the array maxmin.
   * @param CANNON.ConvexHull hull
   * @param CANNON.Vec3 axis
   * @param CANNON.Vec3 pos
   * @param CANNON.Quaternion quat
   * @param array maxmin
   */
  function project(hull,axis,pos,quat,maxmin){
    var n = hull.vertices.length;
    var max = null;
    var min = null;
    var vs = hull.vertices;
    var worldVertex = new CANNON.Vec3();
    for(var i=0; i<n; i++){
      quat.vmult(vs[i],worldVertex);
      worldVertex.vadd(pos,worldVertex);
      var val = worldVertex.dot(axis);
      if(max===null || val>max)
	max = val;
      if(min===null || val<min){
	min = val;
      }
    }

    if(min>max){
      // Inconsistent - swap
      var temp = min;
      min = max;
      max = temp;
    }
    
    // Output
    maxmin[0] = max;
    maxmin[1] = min;
  }

  /**
   * @fn testSepAxis
   * @memberof CANNON.ConvexHull
   * @brief Test separating axis against two hulls. Both hulls are projected onto the axis and the overlap size is returned if there is one.
   * @param CANNON.Vec3 axis
   * @param CANNON.ConvexHull hullB
   * @param CANNON.Vec3 posA
   * @param CANNON.Quaternion quatA
   * @param CANNON.Vec3 posB
   * @param CANNON.Quaternion quatB
   * @return float The overlap depth, or FALSE if no penetration.
   */
  this.testSepAxis = function(axis, hullB, posA, quatA, posB, quatB){
    var maxminA=[], maxminB=[], hullA=this;
    project(hullA, axis, posA, quatB, maxminA);
    project(hullB, axis, posB, quatB, maxminB);
    var maxA = maxminA[0];
    var minA = maxminA[1];
    var maxB = maxminB[0];
    var minB = maxminB[1];

    if(maxA<minB || maxB<minA){
      return false; // Separated
    }
    
    var d0 = maxA - minB;
    var d1 = maxB - minA;
    depth = d0<d1 ? d0:d1;
    return depth;
  }

  /**
   * Find the separating axis between this hull and another
   * @param CANNON.ConvexHull hullB
   * @param CANNON.Vec3 posA
   * @param CANNON.Vec3 quatA
   * @param CANNON.Vec3 posB
   * @param CANNON.Vec3 quatB
   * @param CANNON.Vec3 target The target vector to save the axis in
   * @return bool Returns false if a separation is found, else true
   */
  this.findSeparatingAxis = function(hullB,posA,quatA,posB,quatB,target){
    var dmin = Infinity;
    var hullA = this;
    var curPlaneTests=0;
    var numFacesA = hullA.faces.length;

    // Test normals from hullA
    var faceANormalWS = new CANNON.Vec3();
    for(var i=0; i<numFacesA; i++){
      // Get world face normal
      hullA.faceNormals[i].copy(faceANormalWS);
      quatA.vmult(faceANormalWS,faceANormalWS);
      //posA.vadd(faceANormalWS,faceANormalWS); // Needed?
      //console.log("face normal:",hullA.faceNormals[i].toString(),"world face normal:",faceANormalWS);
      
      var d = hullA.testSepAxis(faceANormalWS, hullB, posA, quatA, posB, quatB);
      if(d===false){
	return false;
      }
      
      if(d<dmin){
	dmin = d;
	faceANormalWS.copy(target);
      }
    }

    // Test normals from hullB
    var WorldNormal = new CANNON.Vec3(); 
    var numFacesB = hullB.faces.length;
    for(var i=0;i<numFacesB;i++){
      hullB.faceNormals[i].copy(WorldNormal);
      quatB.vmult(WorldNormal,WorldNormal);
      //posB.vadd(WorldNormal,WorldNormal);
      //console.log("facenormal",hullB.faceNormals[i].toString(),"world:",WorldNormal.toString());
      curPlaneTests++;
      var d = hullA.testSepAxis(WorldNormal, hullB,posA,quatA,posB,quatB);
      if(d===false){
	return false;
      }
      
      if(d<dmin){
	dmin = d;
	WorldNormal.copy(target);
      }
    }

    var edgeAstart,edgeAend,edgeBstart,edgeBend;
    
    var curEdgeEdge = 0;
    // Test edges
    var WorldEdge0 = new CANNON.Vec3();
    var WorldEdge1 = new CANNON.Vec3();
    var Cross = new CANNON.Vec3();
    for(var e0=0; e0<hullA.uniqueEdges.length; e0++){
      // Get world edge
      hullA.uniqueEdges[e0].copy(WorldEdge0);
      quatA.vmult(WorldEdge0,WorldEdge0);
      //posA.vadd(WorldEdge0,WorldEdge0); // needed?

      //console.log("edge0:",WorldEdge0.toString());

      for(var e1=0; e1<hullB.uniqueEdges.length; e1++){
	hullB.uniqueEdges[e1].copy(WorldEdge1);
	quatB.vmult(WorldEdge1,WorldEdge1);
	//posB.vadd(WorldEdge1,WorldEdge1); // needed?
	//console.log("edge1:",WorldEdge1.toString());
	
	WorldEdge0.cross(WorldEdge1,Cross);

	curEdgeEdge++;
	if(!Cross.almostZero()){
	  Cross.normalize();
	  var dist = hullA.testSepAxis( Cross, hullB, posA,quatA,posB,quatB);
	  if(dist===false){
	    return false;
	  }
	  
	  if(dist<dmin){
	    dmin = dist;
	    Cross.copy(target);
	  }
	}
      }
    }

    var deltaC = new CANNON.Vec3();
    posB.vsub(posA,deltaC);
    if((deltaC.dot(target))>0.0)
      target.negate(target);
    
    return true;
}

  /**
   * @fn clipAgainstHull
   * @memberof CANNON.ConvexHull
   * @brief Clip this hull against another hull
   * @param CANNON.Vec3 posA
   * @param CANNON.Quaternion quatA
   * @param CANNON.ConvexHull hullB
   * @param CANNON.Vec3 posB
   * @param CANNON.Quaternion quatB
   * @param CANNON.Vec3 separatingNormal
   * @param float minDist Clamp distance
   * @param float maxDist
   * @param array result The resulting ContactPoints
   * @see http://bullet.googlecode.com/svn/trunk/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
   * @todo Store contact points?
   */
  this.clipAgainstHull = function(posA,quatA,hullB,posB,quatB,separatingNormal,minDist,maxDist,result){
    if(!(posA instanceof CANNON.Vec3))
      throw new Error("posA must be Vec3");
    if(!(quatA instanceof CANNON.Quaternion))
      throw new Error("quatA must be Quaternion");
    var hullA = this;
    var curMaxDist = maxDist;
    var closestFaceB = -1;
    var dmax = -Infinity;
    var WorldNormal = new CANNON.Vec3();
    for(var face=0; face < hullB.faces.length; face++){
      this.faceNormals[face].copy(WorldNormal);
      quatB.vmult(WorldNormal,WorldNormal);
      posB.vadd(WorldNormal,WorldNormal);

      var d = WorldNormal.dot(separatingNormal);
      if (d > dmax){
	dmax = d;
	closestFaceB = face;
      }
    }
    var worldVertsB1 = [];
    polyB = hullB.faces[closestFaceB];
    var numVertices = polyB.length;
    for(var e0=0; e0<numVertices; e0++){
      var b = hullB.vertices[polyB[e0]];
      var worldb = new CANNON.Vec3();
      b.copy(worldb);
      quatB.vmult(worldb,worldb);
      posB.vadd(worldb,worldb);
      worldVertsB1.push(worldb);
    }
    
    if (closestFaceB>=0)
      this.clipFaceAgainstHull(separatingNormal,
			       posA,
			       quatA,
			       worldVertsB1,
			       minDist,
			       maxDist,
			       result);
  };

  /**
   * Clip a face against a hull
   * @param CANNON.Vec3 separatingNormal
   * @param CANNON.Vec3 posA
   * @param CANNON.Quaternion quatA
   * @param Array worldVertsB1
   * @param float minDist Distance clamping
   * @param float maxDist
   * @param Array result Array to store resulting contact points in
   */
  this.clipFaceAgainstHull = function(separatingNormal, posA, quatA, worldVertsB1, minDist, maxDist,result){
    if(!(separatingNormal instanceof CANNON.Vec3))
       throw new Error("sep normal must be vector");
    if(!(worldVertsB1 instanceof Array))
       throw new Error("world verts must be array");
    minDist = Number(minDist);
    maxDist = Number(maxDist);
    var hullA = this;
    var worldVertsB2 = [];
    var pVtxIn = worldVertsB1;
    var pVtxOut = worldVertsB2;

    // Find the face with normal closest to the separating axis
    var closestFaceA = -1;
    var dmin = Infinity;
    var faceANormalWS = new CANNON.Vec3();
    for(var face=0; face<hullA.faces.length; face++){
      this.faceNormals[face].copy(faceANormalWS);
      quatA.vmult(faceANormalWS,faceANormalWS);
      posA.vadd(faceANormalWS,faceANormalWS);
      
      var d = faceANormalWS.dot(separatingNormal);
      if (d < dmin){
	dmin = d;
	closestFaceA = face;
      }
    }
    if (closestFaceA<0){
      console.log("--- did not find any closest face... ---");
      return;
    }

    // Get the face and construct connected faces
    var polyA = hullA.faces[closestFaceA];
    polyA.connectedFaces = [];
    for(var i=0; i<hullA.faces.length; i++)
      for(var j=0; j<hullA.faces[i].length; j++)
	if(polyA.indexOf(hullA.faces[i][j])!==-1 && // Sharing a vertex
	   i!==closestFaceA && // Not the one we are looking for connections from
	   polyA.connectedFaces.indexOf(i)===-1 // Not already added
	   )
	  polyA.connectedFaces.push(i);
    
    // Clip the polygon to the back of the planes of all faces of hull A, that are adjacent to the witness face
    var numContacts = pVtxIn.length;
    var numVerticesA = polyA.length;
    var edge0 = new CANNON.Vec3();
    var WorldEdge0 = new CANNON.Vec3();
    var worldPlaneAnormal1 = new CANNON.Vec3();
    var planeNormalWS1 = new CANNON.Vec3();
    var res = [];
    for(var e0=0; e0<numVerticesA; e0++){
      var a = hullA.vertices[polyA[e0]];
      var b = hullA.vertices[polyA[(e0+1)%numVerticesA]];
      a.vsub(b,edge0);
      edge0.copy(WorldEdge0);
      quatA.vmult(WorldEdge0,WorldEdge0);
      posA.vadd(WorldEdge0,WorldEdge0);
      this.faceNormals[closestFaceA].copy(worldPlaneAnormal1);//transA.getBasis()* btVector3(polyA.m_plane[0],polyA.m_plane[1],polyA.m_plane[2]);
      quatA.vmult(worldPlaneAnormal1,worldPlaneAnormal1);
      posA.vadd(worldPlaneAnormal1,worldPlaneAnormal1);
      WorldEdge0.cross(worldPlaneAnormal1,planeNormalWS1);
      planeNormalWS1.negate();
      var worldA1 = new CANNON.Vec3();
      a.copy(worldA1);
      quatA.vmult(worldA1,worldA1);
      posA.vadd(worldA1,worldA1);
      var planeEqWS1 = -worldA1.dot(planeNormalWS1);

      if(true){
	var otherFace = polyA.connectedFaces[e0];
	var localPlaneNormal = new CANNON.Vec3();
	this.faceNormals[otherFace].copy(localPlaneNormal);
	var localPlaneEq = planeConstant(otherFace);
	
	var planeNormalWS = new CANNON.Vec3();
	localPlaneNormal.copy(planeNormalWS);
	quatA.vmult(planeNormalWS,planeNormalWS);
	//posA.vadd(planeNormalWS,planeNormalWS);
	var planeEqWS = localPlaneEq - planeNormalWS.dot(posA);
      } else  {
	var planeNormalWS = planeNormalWS1;
	var planeEqWS = planeEqWS1;
      }

      // Clip face against our constructed plane
      console.log("clipping polygon ",printFace(closestFaceA)," against plane ",planeNormalWS, planeEqWS);
      this.clipFaceAgainstPlane(pVtxIn, pVtxOut, planeNormalWS, planeEqWS);
      //console.log(" - clip result: ",pVtxOut);

      // Throw away all clipped points, but save the reamining until next clip
      while(pVtxIn.length)  pVtxIn.shift();
      while(pVtxOut.length) pVtxIn.push(pVtxOut.shift());
    }

    console.log("Resulting points after clip:",pVtxIn);
        
    // only keep contact points that are behind the witness face
    var localPlaneNormal = new CANNON.Vec3();
    this.faceNormals[closestFaceA].copy(localPlaneNormal);
    
    var localPlaneEq = planeConstant(closestFaceA);
    var planeNormalWS = new CANNON.Vec3();
    localPlaneNormal.copy(planeNormalWS);
    quatA.vmult(planeNormalWS,planeNormalWS);
    //posA.vadd(planeNormalWS,planeNormalWS);
    
    var planeEqWS = localPlaneEq - planeNormalWS.dot(posA);
    for (var i=0; i<pVtxIn.length; i++){
      var depth = planeNormalWS.dot(pVtxIn[i]) + planeEqWS; //???
      //console.log("depth calc from normal=",planeNormalWS," and constant "+planeEqWS+" and vertex ",pVtxIn[i]," gives "+depth);
      if (depth <=minDist){
	console.log("clamped: depth="+depth+" to minDist="+(minDist+""));
	depth = minDist;
      }
      
      if (depth <=maxDist){
	var point = pVtxIn[i];
	console.log("Got contact point ",point.toString(),
		    ", depth=",depth,
		    "contact normal=",separatingNormal.toString(),
		    "plane",planeNormalWS.toString(),
		    "planeConstant",planeEqWS);
	//result.push(new CANNON.ContactPoint());
      }
    }
  }
  
  /**
   * Clip a face in a hull against the back of a plane.
   * @param Array inVertices
   * @param Array outVertices
   * @param int face_index
   * @param CANNON.Vec3 planeNormal
   * @param float planeConstant The constant in the mathematical plane equation
   * @todo inVertices is not used - remove?
   */
  this.clipFaceAgainstPlane = function(inVertices,outVertices, planeNormal, planeConstant){
    if(!(planeNormal instanceof CANNON.Vec3))
      throw new Error("planeNormal must be Vec3, "+planeNormal+" given");
    if(!(inVertices instanceof Array))
      throw new Error("invertices must be Array, "+inVertices+" given");
    if(!(outVertices instanceof Array))
      throw new Error("outvertices must be Array, "+outVertices+" given");
    var n_dot_first, n_dot_last;
    var numVerts = inVertices.length;

    if(numVerts < 2)
      return outVertices;
    
    var firstVertex = inVertices[inVertices.length-1];
    var lastVertex =   inVertices[0];

    n_dot_first = planeNormal.dot(firstVertex) + planeConstant;
    
    for(var vi = 0; vi < numVerts; vi++){
      lastVertex = inVertices[vi];
      n_dot_last = planeNormal.dot(lastVertex) + planeConstant;
      if(n_dot_first < 0){
	if(n_dot_last<0){
	  // Start < 0, end < 0, so output lastVertex
	  outVertices.push(lastVertex);
	} else {
	  // Start < 0, end >= 0, so output intersection
	  var newv = new CANNON.Vec3();
	  firstVertex.lerp(lastVertex,
			   n_dot_first * 1.0/(n_dot_first - n_dot_last),
			   newv);
	  outVertices.push(newv);
	}
      } else {
	if(n_dot_last<0){
	  // Start >= 0, end < 0 so output intersection and end
	  var newv = new CANNON.Vec3();
	  firstVertex.lerp(lastVertex,
			   n_dot_first * 1.0/(n_dot_first - n_dot_last),
			   newv);
	  outVertices.push(newv);
	  outVertices.push(lastVertex);
	}
      }
      firstVertex = lastVertex;
      n_dot_first = n_dot_last;
    }
    return outVertices;
  }

  /**
   * Whether the face is visible from the vertex
   * @param array face
   * @param CANNON.Vec3 vertex
   */
  function visible( face, vertex ) {
    var va = that.vertices[ face[ 0 ] ];
    var vb = that.vertices[ face[ 1 ] ];
    var vc = that.vertices[ face[ 2 ] ];

    var n = new CANNON.Vec3();
    normal( va, vb, vc, n );

    // distance from face to origin
    var dist = n.dot( va );

    return n.dot( vertex ) >= dist;
  }

  var that = this;
  function normalOfFace(i,target){
    var f = that.faces[i];
    //console.log(i,that.faces);
    var va = that.vertices[f[0]];
    var vb = that.vertices[f[1]];
    var vc = that.vertices[f[2]];
    return normal(va,vb,vc,target);
  }

  function planeConstant(face_i,target){
    var f = that.faces[face_i];
    var n = that.faceNormals[face_i];
    var v = that.vertices[f[0]];
    var c = -n.dot(v);
    return c;
  }

  /**
   * @brief Get face normal given 3 vertices
   * @param CANNON.Vec3 va
   * @param CANNON.Vec3 vb
   * @param CANNON.Vec3 vc
   * @param CANNON.Vec3 target
   * @todo unit test?
   */
  function normal( va, vb, vc, target ) {
    var cb = new CANNON.Vec3();
    var ab = new CANNON.Vec3();

    vb.vsub(va,ab);
    vc.vsub(vb,cb);
    cb.cross(ab,target);

    if ( !target.isZero() ) {
      target.normalize();
    }
  }

  function printFace(i){
    var f = that.faces[i], s = "";
    for(var j=0; j<f.length; j++)
      s += " ("+that.vertices[f[j]]+")";
    return s;
  }

  /**
   * Detect whether two edges are equal.
   * Note that when constructing the convex hull, two same edges can only
   * be of the negative direction.
   * @return bool
   */
  function equalEdge( ea, eb ) {
    return ea[ 0 ] === eb[ 1 ] && ea[ 1 ] === eb[ 0 ]; 
  }

  /**
   * Create a random offset between -1e-6 and 1e-6.
   * @return float
   */
  function randomOffset() {
    return ( Math.random() - 0.5 ) * 2 * 1e-6;
  }
};

CANNON.ConvexHull.prototype = new CANNON.Shape();
CANNON.ConvexHull.prototype.constructor = CANNON.ConvexHull;