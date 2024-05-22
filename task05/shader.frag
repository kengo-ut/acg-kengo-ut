#version 120

// see the GLSL 1.2 specification:
// https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.1.20.pdf

// signed distance function of a cylinder the axis is aligned to z-direction
// code from: https://iquilezles.org/articles/distfunctions/
float sdCappedCylinder( vec3 p, float h, float r )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - vec2(r,h);
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

// signed distance function of an axis aligned box.
// code from: https://iquilezles.org/articles/distfunctions/
float sdBox( vec3 p, vec3 b )
{
  vec3 q = abs(p) - b;
  return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}

// signed distance function of a sphere
// code from: https://iquilezles.org/articles/distfunctions/
float sdSphere( vec3 p, float s )
{
  return length(p)-s;
}

// here is the parameter use to draw the objects
float len_cylinder = 0.8; // length of the cylinder
float rad_cylinder = 0.45; // radius of the cylinder
float rad_sphere = 0.8; // radius of the sphere
float box_size = 0.6; // size of box

/// singed distance function at the position `pos`
float SDF(vec3 pos)
{
  float d0 = sdCappedCylinder(pos, len_cylinder, rad_cylinder);
  // write some code to combine the signed distance fields above to design the object described in the README.md
  // SDFs of Rotated Cylinders
  vec3 pos_rot1 = vec3(pos.y, pos.x, pos.z);
  float d1 = sdCappedCylinder(pos_rot1, len_cylinder, rad_cylinder);
  vec3 pos_rot2 = vec3(pos.x, pos.z, pos.y);
  float d2 = sdCappedCylinder(pos_rot2, len_cylinder, rad_cylinder);
  // SDF of the Box
  float d3 = sdBox(pos, vec3(box_size, box_size, box_size));
  // SDF of the Sphere
  float d4 = sdSphere(pos, rad_sphere);

  // CSG operations
  // union of the Cylinders
  float d5 = min(min(d0, d1), d2);
  // intersection of the Box and the Sphere
  float d6 = max(d3, d4);
  // difference of the union and the intersection
  float d7 = max(-d5, d6);
  return d7;
  // return d0; // comment out and define new distance
}

/// RGB color at the position `pos`
vec3 SDF_color(vec3 pos)
{
  // write some code below to return color (RGB from 0 to 1) to paint the object describe in README.md
  // return vec3(0., 1., 0.); // comment out and define new color
  float d0 = sdCappedCylinder(pos, len_cylinder, rad_cylinder);
  vec3 pos_rot1 = vec3(pos.y, pos.x, pos.z);
  float d1 = sdCappedCylinder(pos_rot1, len_cylinder, rad_cylinder);
  vec3 pos_rot2 = vec3(pos.x, pos.z, pos.y);
  float d2 = sdCappedCylinder(pos_rot2, len_cylinder, rad_cylinder);
  float d3 = sdBox(pos, vec3(box_size, box_size, box_size));
  float d4 = sdSphere(pos, rad_sphere);
  float d5 = min(min(d0, d1), d2);
  // float d6 = max(d3, d4);

  // SDF of pos == max(-d5, d6) == max(d3, d4, -d5)
  // if SDF of pos ==  d3, the pos is nearest to the surface of the box
  // if SDF of pos ==  d4, the pos is nearest to the surface of the sphere
  // if SDF of pos == -d5, the pos is nearest to the surface of the union of the cylinders
  // so we compare SDF of pos with d3, d4, and -d5
  float sdf_pos = SDF(pos);
  if (sdf_pos == d3) {
    // the color of the box
    return vec3(1., 0., 0.);
  }
  else if (sdf_pos == d4) {
    // the color of the sphere
    return vec3(0., 0., 1.);
  }
  else {
    // the color of the union of the cylinders
    return vec3(0., 1., 0.);
  }
}

uniform float time; // current time given from CPU

void main()
{
  // camera position
  vec3 cam_pos = normalize( vec3(sin(time),cos(time),0.5*sin(time)) );

  // local frame defined on the cameera
  vec3 frame_z = cam_pos;
  vec3 frame_x = normalize(cross(vec3(0,0,1),frame_z));
  vec3 frame_y = cross(frame_z,frame_x);

  // gl_FragCoord: the coordinate of the pixel
  // left-bottom is (0,0), right-top is (W,H)
  // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
  vec2 scr_xy = gl_FragCoord.xy / vec2(500,500) * 2.0 - vec2(1,1); // canonical screen position [-1,+1] x [-1,+1]
  vec3 src = frame_x * scr_xy.x + frame_y * scr_xy.y + frame_z * 1;  // source of ray from pixel
  vec3 dir = -frame_z;  // direction of ray (looking at the origin)

  vec3 pos_cur = src; // the current ray position
  for(int itr=0;itr<60;++itr){
    float s0 = SDF(pos_cur);
    if( s0 < 0.0 ){ // ray starting from inside the object
      gl_FragColor = vec4(1, 0, 0, 1); // paint red
      return;
    }
    if( s0 < 1.0e-3 ){ // the ray hit the implicit surfacee
      float eps = 1.0e-3;
      float sx = SDF(pos_cur+vec3(eps,0,0))-s0; // finite difference x-direction
      float sy = SDF(pos_cur+vec3(0,eps,0))-s0; // finite difference x-direction
      float sz = SDF(pos_cur+vec3(0,0,eps))-s0; // finite difference y-direction
      vec3 nrm = normalize(vec3(sx,sy,sz)); // normal direction
      float coeff = -dot(nrm, dir); // Lambersian reflection. The light is at the camera position.
      vec3 color = SDF_color(pos_cur);
      gl_FragColor = vec4(coeff*color, 1);
      return;
    }
    pos_cur += s0 * dir; // advance ray
  }
  gl_FragColor = vec4(0.9, 0.9, 1.0, 1); // ray doesn't hit the object
}
